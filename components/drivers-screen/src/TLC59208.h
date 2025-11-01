#ifndef TLC_H_
#define TLC_H_

/*
    The TLC Code

    Adapted from "https://github.com/Plastic-Scanner"
*/

class TLC59208 {
public:
    TLC59208() {}
    void init();
    void toggleLED(int x, bool on);

    esp_err_t ret;
};

#endif