#ifndef SCREEN_H_
#define SCREEN_H_
#include <string>
#include <ctype.h>

/*
    Contains the Screen Code

    A lot is byte and bit manipulation stuff
*/

class LCD_I2C{
public:
    LCD_I2C();

    void init(uint8_t addr, uint8_t dataPin, uint8_t clockPin, uint8_t cols, uint8_t rows);
    void setCursor(uint8_t col, uint8_t row);
    void home(void);
    void clear(void);
    void writeChar(char c) const;
    void print(std::string str) const;
    void print(int i) const; 
    void print(char c) const;

    void println(std::string str);
    void println(int i);
    void println(char c);

    void print_pred(int pred, float confidence);
private:
    uint8_t cur_col,cur_row;

};


#endif