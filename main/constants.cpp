#include "constants.h"

// Number of plastic types
int plastic_types_tot = 14;

// Dictionary mapping TFLite results to actual plastics
std::vector<std::string> plastic_type_mapping = {
    "ABS",
    "ASA",
    "HDPE",
    "LDPE",
    "Other",
    "PACF",
    "PC",
    "PCCF",
    "PET",
    "PLA",
    "PMMA",
    "PP",
    "PS",
    "PVC"
};