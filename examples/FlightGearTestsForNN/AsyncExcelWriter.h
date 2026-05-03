#pragma once

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <string>
#include <OpenXLSX.hpp>
#include <iostream>
#include <cassert>

/**
 * @brief Asynchronous Excel writer for flight data logging
 *
 * This class provides a thread-safe way to write flight data to an Excel file
 * without blocking the main control loop. Data is queued and written in batches.
 */
class AsyncExcelWriter {
private:
    static constexpr uint32_t BATCH_SIZE = 100;  // Configurable batch size

    struct Batch {
        std::vector<std::vector<double>> rows;
        int start_row = 0;

        Batch() = default;
        Batch(int sr) : start_row(sr) {}
    };

    std::queue<std::vector<double>> m_queue;
    std::mutex m_mutex;
    std::condition_variable m_cond;
    std::thread m_worker;
    bool m_stop = false;

    OpenXLSX::XLDocument m_doc;
    OpenXLSX::XLWorksheet m_wks;
    std::string m_filename;
    std::vector<std::string> m_headers;
    int m_row = 2; // Row 1 is for headers

    void loop() {
        while (true) {
            std::vector<std::vector<double>> batch_data;
            int batch_start_row = 0;

            {
                std::unique_lock<std::mutex> lock(m_mutex);

                // Wait until we have BATCH_SIZE rows or shutdown + remaining data
                // Use wait_for with timeout to prevent indefinite blocking
                bool wait_result = m_cond.wait_for(lock, std::chrono::seconds(5), [this] {
                    return m_stop || m_queue.size() >= BATCH_SIZE;
                });

                if (m_queue.empty() && m_stop) {
                    break;
                }

                // Take up to BATCH_SIZE rows
                batch_start_row = m_row;
                size_t take = std::min(static_cast<size_t>(BATCH_SIZE), m_queue.size());
                batch_data.reserve(take);

                for (size_t i = 0; i < take; ++i) {
                    if (m_queue.empty()) break;  // Safety check
                    batch_data.emplace_back(std::move(m_queue.front()));
                    m_queue.pop();
                    ++m_row;
                }

                // If data remains, wake up again on next addToQueue
                if (!m_queue.empty() && !m_stop) {
                    m_cond.notify_one();
                }
            }

            // Write batch to Excel (without holding mutex!)
            if (!batch_data.empty()) {
                try {
                    writeBatch(batch_start_row, batch_data);
                    m_doc.save();  // Save once per batch
                } catch (const std::exception& e) {
                    std::cerr << "Error writing Excel data: " << e.what() << std::endl;
                }
            }
        }

        // Final flush of remaining data (< BATCH_SIZE)
        flushRemaining();
    }

    void writeBatch(int start_row, const std::vector<std::vector<double>>& batch) {
        assert(!batch.empty());
        size_t cols = m_headers.size();

        for (size_t r = 0; r < batch.size(); ++r) {
            const auto& row = batch[r];
            if (row.size() != cols) {
                std::cerr << "Warning: row size mismatch! Expected " << cols
                          << ", got " << row.size() << " at row " << (start_row + r) << std::endl;
                continue;
            }

            for (size_t c = 0; c < cols; ++c) {
                m_wks.cell(OpenXLSX::XLCellReference(start_row + static_cast<int>(r), static_cast<uint16_t>(c + 1)))
                .value() = row[c];
            }
        }
    }

    void flushRemaining() {
        std::vector<std::vector<double>> remaining;
        int start_row = 0;

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (m_queue.empty()) return;
            start_row = m_row;
            remaining.reserve(m_queue.size());
            while (!m_queue.empty()) {
                remaining.emplace_back(std::move(m_queue.front()));
                m_queue.pop();
                ++m_row;
            }
        }

        if (!remaining.empty()) {
            writeBatch(start_row, remaining);
            m_doc.save();
        }
    }

public:
    /**
     * @brief Constructor
     * @param filename Output Excel file name
     * @param headers Column headers for the data
     */
    AsyncExcelWriter(const std::string& filename, const std::vector<std::string>& headers)
        : m_filename(filename), m_headers(headers) {

        try {
            // Validate filename
            if (filename.empty()) {
                throw std::invalid_argument("Filename cannot be empty");
            }
            
            // Check for invalid characters in filename
            const std::string invalid_chars = "<>:\"/\\|?*";
            if (filename.find_first_of(invalid_chars) != std::string::npos) {
                throw std::invalid_argument("Filename contains invalid characters");
            }

            m_doc.create(m_filename);
            m_wks = m_doc.workbook().worksheet("Sheet1");

            for (size_t i = 0; i < m_headers.size(); ++i) {
                m_wks.cell(OpenXLSX::XLCellReference(1, static_cast<uint16_t>(i + 1))).value() = m_headers[i];
            }

            m_doc.save();
            m_worker = std::thread(&AsyncExcelWriter::loop, this);

        } catch (const std::exception& e) {
            std::cerr << "Failed to create Excel file '" << filename << "': " << e.what() << std::endl;
            throw;
        }
    }

    ~AsyncExcelWriter() {
        try {
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_stop = true;
            }
            m_cond.notify_all();
            
            // Simply join the worker thread - the caller should have waited
            // for data to be written before destroying this object
            if (m_worker.joinable()) {
                m_worker.join();
            }

            if (m_doc.isOpen()) {
                try {
                    m_doc.save();
                    m_doc.close();
                } catch (const std::exception& e) {
                    std::cerr << "Error closing Excel file: " << e.what() << std::endl;
                }
            }
        } catch (...) {
            // Suppress exceptions in destructor
        }
    }

    /**
     * @brief Add a row of data to the write queue
     * @param data Container of double values to write
     */
    template<typename Container>
    void addToQueue(Container&& data) {
        static_assert(std::is_same_v<std::decay_t<typename Container::value_type>, double>,
                      "Container must contain values convertible to double");

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            // Check queue size to prevent unbounded memory growth
            if (m_queue.size() > 10000) {
                std::cerr << "Warning: Excel write queue is full, dropping data" << std::endl;
                return;
            }
            m_queue.emplace(std::begin(data), std::end(data));
        }
        m_cond.notify_one();
    }

    // Disable copying
    AsyncExcelWriter(const AsyncExcelWriter&) = delete;
    AsyncExcelWriter& operator=(const AsyncExcelWriter&) = delete;
};
