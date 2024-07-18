#include "bag_extractor/Utils.hpp"

#include <sstream>
namespace bag_extractor
{

    namespace utils
    {

        std::string get_file_name(const std::string folder, const std::string device, const double timestamp, const std::string extension)
        {
            std::string timestamp_str = fmt::format("{:0<10.9f}", timestamp);
            std::string filename = extension;
            // filename = device + "-" + timestamp_str + filename;
            filename = timestamp_str + filename;

            // filename = "./" + folder + filename;
            filename = folder + filename;

            return filename;
        }

        std::string get_device(const std::string folder)
        {
            std::string delimiter = "/";

            std::string device = folder.substr(folder.find(delimiter, 0) + 1, folder.size() - 8);

            return device;
        }

        std::vector<std::string> split_strings(std::string const &str, char const &delimiter)
        {
            std::stringstream ss(str);
            std::string item;
            std::vector<std::string> output;
            while (std::getline(ss, item, delimiter))
            {
                std::cout << "Introduced item: " << item.c_str() << std::endl;
                output.push_back(item);
            }
            return output;
        }

    } // namespace utils

} // namespace bag_extractor