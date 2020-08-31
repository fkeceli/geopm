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
#include "EditDistPeriodicityDetector.hpp"

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

#include "record.hpp"
#include "geopm_debug.hpp"

namespace geopm
{
    EditDistPeriodicityDetector::EditDistPeriodicityDetector(int history_buffer_size, bool squash_records)
        : m_history_buffer(history_buffer_size)
        , m_repeat_count(history_buffer_size)
        , m_history_buffer_size(history_buffer_size)
        , m_score(-1)
        , m_record_count(0)
        , m_DP(history_buffer_size * history_buffer_size * history_buffer_size)
        , m_squash_records(squash_records)
        , m_last_event(0)
        , m_last_event_count(0)
        // This is right though because in the case where event squashing
        // is being used, calc_period will never be called when all the
        // events are identical, and in this case, the period is 1.
        , m_period(1)
    {
        if (squash_records) {
            m_period = 1;
        }
        else {
            m_period = -1;
        }
    }

    bool EditDistPeriodicityDetector::update(const record_s &record)
    {
        if (record.event == EVENT_REGION_ENTRY) {
            if (m_squash_records) {
                if (m_last_event == record.signal) {
                    ++m_last_event_count;
                    return false;
                }
                else {
                    bool return_val = false;
                    if (m_last_event_count > 0) {
                        m_history_buffer.insert(record.signal);
                        m_repeat_count.insert(m_last_event_count);
                        ++m_record_count;
                        calc_period();
                        return_val = true;
                    }
                    m_last_event = record.signal;
                    m_last_event_count = 1;
                    return return_val;
                }
            } else {
                m_history_buffer.insert(record.signal);
                ++m_record_count;
                calc_period();
                return true;
            }
        } else {
            return false;
        }
    }

    void EditDistPeriodicityDetector::Dset(int ii, int jj, int mm, uint32_t val) {
        m_DP[((ii % m_history_buffer_size)  * m_history_buffer_size
            + (jj % m_history_buffer_size)) * m_history_buffer_size
            + (mm % m_history_buffer_size)] = val;
    }

    uint32_t EditDistPeriodicityDetector::Dget(int ii, int jj, int mm) {
        if (ii <= m_record_count - m_history_buffer_size) {
            // This value is supposed to be INF but not so large that it gets wrapped around when
            // a small value is added to it.
            return std::numeric_limits<uint32_t>::max() / 2;
        }
        if (jj >= m_history_buffer_size) {
            // This value is supposed to be INF but not so large that it gets wrapped around when
            // a small value is added to it.
            return std::numeric_limits<uint32_t>::max() / 2;
        }
        if (mm <= m_record_count - m_history_buffer_size) {
            // This value is supposed to be INF but not so large that it gets wrapped around when
            // a small value is added to it.
            return std::numeric_limits<uint32_t>::max() / 2;
        }
        return m_DP[((ii % m_history_buffer_size)  * m_history_buffer_size
                   + (jj % m_history_buffer_size)) * m_history_buffer_size
                   + (mm % m_history_buffer_size)];
     }

    void EditDistPeriodicityDetector::calc_period(void)
    {
        if (m_record_count < 2) {
            return;
        }

        int num_recs_in_hist = m_history_buffer.size();

        for (int ii = std::max({0, m_record_count - m_history_buffer_size}); ii < m_record_count; ++ii) {
            Dset(ii, 0, m_record_count - 1, 0);
        }
        for (int mm = std::max({0, m_record_count - m_history_buffer_size}); mm < m_record_count; ++mm) {
            Dset(0, m_record_count - mm, mm, m_record_count - mm);
        }

        for (int mm = std::max({1, m_record_count-m_history_buffer_size}); mm < m_record_count; ++mm) {
            for (int ii = std::max({1, m_record_count-m_history_buffer_size}); ii < mm+1; ++ii) {
                int term;
                if (m_record_count - (ii - 1) <= num_recs_in_hist) {
                    if (m_squash_records) {
                        if (m_history_buffer.value(num_recs_in_hist - (m_record_count - (ii - 1))) ==
                            m_history_buffer.value(num_recs_in_hist - 1)) {
                            term = abs(m_repeat_count.value(num_recs_in_hist - (m_record_count - (ii - 1))) - m_repeat_count.value(num_recs_in_hist - 1));
                        }
                        else {
                            term = m_repeat_count.value(num_recs_in_hist - (m_record_count - (ii - 1))) + m_repeat_count.value(num_recs_in_hist - 1);
                        }
                    }
                    else {
                        term = (m_history_buffer.value(num_recs_in_hist - (m_record_count - (ii - 1))) !=
                                m_history_buffer.value(num_recs_in_hist - 1)) ?
                            2 : 0;
                    }
                } else {
                    term = m_squash_records ? 2 * m_repeat_count.value(num_recs_in_hist - 1) : 2;
                }
                Dset(ii, m_record_count - mm, mm,
                        std::min({Dget(ii - 1, m_record_count - mm    , mm) + 1,
                                  Dget(ii    , m_record_count - mm - 1, mm) + 1,
                                  Dget(ii - 1, m_record_count - mm - 1, mm) + term}));
            }
        }

        int mm = std::max({(int)(m_record_count / 2.0 + 0.5), m_record_count - m_history_buffer_size});
        int bestm = mm;
        uint32_t bestval = Dget(mm, m_record_count - mm, mm);
        ++mm;
        for(; mm < m_record_count; ++mm) {
            uint32_t val = Dget(mm, m_record_count - mm, mm);
            if(val < bestval) {
                bestval = val;
                bestm = mm;
            }
        }

        m_score = bestval;
        // Originally this was:
        //      m_period = n - bestm + 1;
        // However since the algorithm find the bestm with the lowest index it will
        // return a string with a repeating pattern in it. For example:
        //     A B A B A B ...
        // find_min_match will find the smallest repeating pattern in it: A B
        m_period = find_smallest_repeating_pattern(num_recs_in_hist - (m_record_count - bestm));
    }

    int EditDistPeriodicityDetector::get_period(void) const
    {
        return m_period;
    }

    int EditDistPeriodicityDetector::get_score(void) const
    {
        return m_score;
    }

    uint64_t EditDistPeriodicityDetector::get_history_value(int index) const
    {
        return m_history_buffer.value(index - 1);
    }

    int EditDistPeriodicityDetector::num_records(void) const
    {
        return m_record_count;
    }

    int EditDistPeriodicityDetector::find_smallest_repeating_pattern(int slice_start) const
    {
        if (m_history_buffer.size() == slice_start) {
            return 1;
        }
        std::vector<uint64_t> recs = m_history_buffer.make_vector(
            slice_start, m_history_buffer.size());

        std::vector<int> reps;
        if (m_squash_records) {
            reps = m_repeat_count.make_vector(
                    slice_start, m_history_buffer.size());
        }

        int result = recs.size();
        bool perfect_match = false;
        int div_max = (recs.size() / 2) + 1;
        for (int div = 1; !perfect_match && div < div_max; ++div) {
            if (recs.size() % div == 0) {
                perfect_match = true;
                int group_max = recs.size() / div;
                for (int group = 1; perfect_match && group < group_max; ++group) {
                    auto cmp1_begin = recs.begin() + div * (group - 1);
                    auto cmp1_end = recs.begin() + div * group;
                    auto cmp2_begin = cmp1_end;
                    if (!std::equal(cmp1_begin, cmp1_end, cmp2_begin)) {
                        perfect_match = false;
                    }
                    if (perfect_match && m_squash_records) {
                        auto cmp1_begin = reps.begin() + div * (group - 1);
                        auto cmp1_end = reps.begin() + div * group;
                        auto cmp2_begin = cmp1_end;
                        if (!std::equal(cmp1_begin, cmp1_end, cmp2_begin)) {
                            perfect_match = false;
                        }
                    }
                }
                if (perfect_match) {
                    result = div;
                }
            }
        }
        return result;
    }
}
