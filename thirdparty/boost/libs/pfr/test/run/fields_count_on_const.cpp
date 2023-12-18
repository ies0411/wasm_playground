// Copyright (c) 2018-2022 Antony Polukhin
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

#include <boost/pfr/detail/fields_count.hpp>

struct some_struct {
    int i;
    int j;
};

int main() {
    static_assert(boost::pfr::detail::fields_count<const some_struct>() == 2, "");
}
