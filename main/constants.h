#ifndef CONSTANTS_H  // Include guard to prevent multiple inclusion
#define CONSTANTS_H

/*
    Contains some constants used in this file
*/

#include <vector>
#include <string>

// Declare the number of plastic types
extern int plastic_types_tot;

// Declare the dictionary mapping TFLite results to actual plastics
extern std::vector<std::string> plastic_type_mapping;

#endif  // CONSTANTS_H