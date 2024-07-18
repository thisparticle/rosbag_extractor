#pragma once

// STD
#include <string>
#include <vector>
#include <fstream>
#include <iomanip>
#include <iostream>

// fmt
#include <fmt/core.h>

namespace bag_extractor
{

namespace utils
{

//! Function to get the name of a file with the number and folder route (Specific use case)
std::string get_file_name(const std::string folder, const std::string device, const double timestamp, const std::string extension);

//! Function to extract the device from the folder to save the files (Specific use case)
// It assumes folder/device/ name convention.
std::string get_device(const std::string folder);

//! Function to get several strings from a string list and a separator
std::vector<std::string> split_strings(std::string const &str, char const &delimiter);

} // namespace utils

} // namespace bag_extractor