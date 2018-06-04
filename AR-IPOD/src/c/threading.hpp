//
//  threading.hpp
//  AR-IPOD
//
//  Created by Remi Decelle on 16/05/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#ifndef threading_hpp
#define threading_hpp

#include <stdio.h>
#include <cstddef>
#include <thread>
#include <algorithm>
#include <vector>

template<typename Iterator, class Function>
void parallel_for(const Iterator& first, const Iterator& last, Function&& f, const int nthreads = 4, const int threshold = 1000)
{
    const auto group = std::max(std::max(ptrdiff_t(1), ptrdiff_t(std::abs(threshold))), ((last-first))/std::abs(nthreads));
    std::vector<std::thread> threads;
    threads.reserve(nthreads);
    Iterator it = first;
    for (; it < last - group; it = std::min(it + group, last))
    {
        threads.push_back(std::thread([=,&f](){std::for_each(it, std::min(it+group, last), f);}));
    }
    // last steps while we wait for other threads
    std::for_each(it, last, f);
    
    std::for_each(threads.begin(), threads.end(), [](std::thread& x){x.join();});
}

#endif /* threading_hpp */
