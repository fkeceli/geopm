/*
 * Copyright (c) 2015, 2016, 2017, 2018, 2019, 2020, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "config.h"

#include <exception>

#include <iostream>
#include <time.h>

#include "RecordAnalysisFilter.hpp"
#include "Exception.hpp"
#include "Helper.hpp"
#include "record.hpp"
#include "geopm_debug.hpp"
#include "geopm_hash.h"

namespace geopm
{

    static int parse_report_interval(const std::string &name)
    {
        double report_interval_secs;
        RecordAnalysisFilter::parse_name(name, report_interval_secs);
        return report_interval_secs;
    }

    RecordAnalysisFilter::RecordAnalysisFilter(const std::string &name)
    : RecordAnalysisFilter(parse_report_interval(name))
    {

    }

    RecordAnalysisFilter::RecordAnalysisFilter(double report_interval_secs)
    : m_histogram()
    , m_report_interval_secs(report_interval_secs)
    {
        time(&m_last_sample_time);
    }

    std::vector<record_s> RecordAnalysisFilter::filter(const record_s &record)
    {
        std::vector<record_s> result;
        result.push_back(record);
        if (record.event == EVENT_REGION_ENTRY) {
            if (m_histogram.find(record.signal) == m_histogram.end()) {
                m_histogram[record.signal] = 1;
            }
            else {
                ++m_histogram[record.signal];
            }
        }

        time_t now;
        time(&now);

        double time_passed = difftime(now, m_last_sample_time);

        if (time_passed > m_report_interval_secs) {
            std::cout << "New histogram report at time " << ctime(&now);
            for (auto &it : m_histogram) {
                std::cout << it.first
                          << ':'
                          << it.second
                          << std::endl;
            }
            m_last_sample_time = now;
            m_histogram.clear();
        }

        return result;
    }

    void RecordAnalysisFilter::parse_name(const std::string &name, double &report_interval_secs)
    {
        auto pieces = string_split(name, ",");
        GEOPM_DEBUG_ASSERT(pieces.size() > 0, "string_split() failed.");

        // Default values
        report_interval_secs = 5;

        if (pieces[0] != "record_analysis") {
            throw Exception("Unknown filter name", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (pieces.size() > 1) {
            try {
                report_interval_secs = std::stod(pieces[1]);
            }
            catch (const std::exception &ex) {
                throw Exception("RecordAnalysisFilter::parse_name(): invalid report interval",
                                GEOPM_ERROR_INVALID, __FILE__, __LINE__);
            }
        }
        if (pieces.size() > 2) {
            throw Exception("Too many commas in filter name", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
    }
}
