idf_component_register(
    INCLUDE_DIRS
        # "src"
        "."
    SRCS
        "src/Screen.cpp" "src/NAU7802.cpp" "src/TLC59208.cpp"
    REQUIRES
        driver
        esp_timer
)