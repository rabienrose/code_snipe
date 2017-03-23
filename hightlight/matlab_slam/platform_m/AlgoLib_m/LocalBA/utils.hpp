//
//  utils.hpp
//  testSLAMTracker
//
//  Created by zhaiq on 7/21/16.
//  Copyright © 2016 zhaiq. All rights reserved.
//

#ifndef utils_hpp
#define utils_hpp

#include <stdio.h>
#include <unordered_map>
#include "typeDef.hpp"

void save(const char* file, const std::unordered_map<long, ygomi::Frame>& frames);

void save(const char* file, const std::unordered_map<long, ygomi::MapPoint>& mappoints);

#endif /* utils_hpp */
