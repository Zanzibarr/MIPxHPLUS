#include "../include/utils.hpp"

// ##################################################################### //
// ############################## BITFIELD ############################# //
// ##################################################################### //

BitField::BitField(unsigned int size) { this -> len = size; this -> field = new char[(int)((size+7)/8)](); }

BitField::~BitField() { delete[] this -> field; this -> field = nullptr; }

void BitField::set(const unsigned int i) {
    
    my::assert(i < this -> len, "[BITFIELD SET]");
    this -> field[i/8] |= (1 << i%8);
    
}

void BitField::unset(const unsigned int i) {
    
    my::assert(i < this -> len, "[BITFIELD UNSET]");
    this -> field[i/8] &= ~(1 << i%8);
    
}

bool BitField::operator[](const unsigned int i) const {
    
    my::assert(i < this -> len, "BITFIELD OPERATOR[]]");
    return this -> field[i/8] & (1 << i%8);
    
}

BitField BitField::operator&(const BitField bf) const {

    my::assert(this -> len == bf.len, "[BITFIELD OPERATOR&]");
    BitField ret = BitField(this -> len);
    for (int i = 0; i < this -> len; i++) if ((this -> field[i/8] & (1 << i%8)) && bf[i]) ret.set(i);
    return ret;

}

bool BitField::operator==(const BitField bf) const {

    my::assert(this -> len == bf.len, "[BITFIELD OPERATOR==]");
    for (int i = 0; i < (this -> len + 7) / 8; i++) if (this -> field[i] != bf.field[i]) return false;
    return true;

}

unsigned int BitField::size() const { return this -> len; }

std::string BitField::view() const {

    std::string ret;
    for (int i = 0; i < this -> len; i++) ret.append(std::to_string(this -> operator[](i)));
    return ret;

}

// ##################################################################### //
// ############################### LOGGER ############################## //
// ##################################################################### //

void Logger::_format_output(const char* str, va_list ptr, FILE* log_file) const {

    double elapsed_time = this -> get_time();
    fprintf(stdout, "%8.3f : ", elapsed_time);
    fprintf(log_file, "%8.3f : ", elapsed_time);

    // char array to store token 
    char token[1000]; 
    // index of where to store the characters of str in token 
    int k = 0; 

    // parsing the formatted string 
    for (int i = 0; str[i] != '\0'; i++) { 

        token[k++] = str[i];

        if (str[i + 1] == '%' || str[i + 1] == '\0') {

            token[k] = '\0'; 
            k = 0;
            if (token[0] != '%') {

                fprintf(stdout, "%s", token); // printing the whole token if it is not a format specifier
                fprintf(log_file, "%s", token);

            } else { 

                int j = 1; 
                char ch1 = 0; 

                // this loop is required when printing formatted value like 0.2f, when ch1='f' loop ends 
                while ((ch1 = token[j++]) < 58) {}

                if (ch1 == 'i' || ch1 == 'd' || ch1 == 'u' || ch1 == 'h') { // for integers

                    int value = va_arg(ptr, int);
                    fprintf(stdout, token, value);
                    fprintf(log_file, token, value);

                } else if (ch1 == 'c') { // for chars

                    char value = va_arg(ptr, int);
                    fprintf(stdout, token, value);
                    fprintf(log_file, token, value);

                } else if (ch1 == 'f') { // for float values 

                    double value = va_arg(ptr, double);
                    fprintf(stdout, token, value);
                    fprintf(log_file, token, value);

                } else if (ch1 == 'l') { // for long values

                    char ch2 = token[2];

                    if (ch2 == 'u' || ch2 == 'd' || ch2 == 'i') { // for long int values 

                        long value = va_arg(ptr, long);
                        fprintf(stdout, token, value);
                        fprintf(log_file, token, value);

                    } else if (ch2 == 'f') {  // for double values 

                        double value = va_arg(ptr, double);
                        fprintf(stdout, token, value);
                        fprintf(log_file, token, value);

                    } 

                } else if (ch1 == 'L') { 

                    char ch2 = token[2]; 

                    if (ch2 == 'u' || ch2 == 'd' || ch2 == 'i') { // for long long int values 

                        long long value = va_arg(ptr, long long);
                        fprintf(stdout, token, value);
                        fprintf(log_file, token, value);

                    } 

                    else if (ch2 == 'f') { // for long double values 

                        long double value = va_arg(ptr, long double);
                        fprintf(stdout, token, value);
                        fprintf(log_file, token, value);

                    } 

                } else if (ch1 == 's') { // for string values 

                    char* value = va_arg(ptr, char*);
                    fprintf(stdout, token, value);
                    fprintf(log_file, token, value);

                } else { // print the whole token if no case is matched 

                    fprintf(stdout, "%s", token);
                    fprintf(log_file, "%s", token);

                } 

            } 
        
        }

    }
}

Logger::Logger(const std::string run_title, const std::string log_name) {

    this -> reset_timer();

    this -> log_file_name = HPLUS_LOG_DIR"/" + log_name;
    FILE* log_file = fopen(this -> log_file_name.c_str(), "a");
    fprintf(log_file, "\n--------------------------------\n%s\n--------------------------------\n", run_title.c_str());
    fclose(log_file);

}

void Logger::reset_timer() { this -> s_timer = std::chrono::steady_clock::now(); }

double Logger::get_time() const { return ((double) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - this -> s_timer).count()) / 1000; }

void Logger::print_info(const char* str, ...) const {

    #if HPLUS_VERBOSE < 10
    return;
    #endif

    // logging file
    FILE* log_file = fopen(this -> log_file_name.c_str(), "a");

    // initializing list pointer 
    va_list ptr; 
    va_start(ptr, str);

    fprintf(stdout, "\033[92m\033[1m[ INFO  ]\033[0m -- ");
    fprintf(log_file, "[ INFO  ] -- ");

    // printing and logging the formatted message
    _format_output(str, ptr, log_file);

    fprintf(stdout, "\n");
    fprintf(log_file, "\n");
    
    fclose(log_file);

    // ending traversal 
    va_end(ptr);

}

void Logger::print_warn(const char* str, ...) const {

    #if HPLUS_VERBOSE < 1
    return;
    #endif

    // logging file
    FILE* log_file = fopen(this -> log_file_name.c_str(), "a");

    // initializing list pointer 
    va_list ptr; 
    va_start(ptr, str);

    fprintf(stdout, "\033[93m\033[1m[ WARN  ]\033[0m -- ");
    fprintf(log_file, "[ WARN  ] -- ");

    // printing and logging the formatted message
    _format_output(str, ptr, log_file);

    fprintf(stdout, "\n");
    fprintf(log_file, "\n");
    
    fclose(log_file);

    // ending traversal 
    va_end(ptr);

}

void Logger::raise_error(const char* str, ...) const {

    // logging file
    FILE* log_file = fopen(this -> log_file_name.c_str(), "a");

    // initializing list pointer 
    va_list ptr; 
    va_start(ptr, str);

    fprintf(stdout, "\033[91m\033[1m[ ERROR ]\033[0m -- ");
    fprintf(log_file, "[ ERROR ] -- ");

    // printing and logging the formatted message
    _format_output(str, ptr, log_file);

    fprintf(stdout, "\n");
    fprintf(log_file, "\n");
    
    fclose(log_file);

    // ending traversal 
    va_end(ptr);

    exit(1);

}

// ##################################################################### //
// ############################ MY NAMESPACE ########################### //
// ##################################################################### //

void my::split(const std::string str, std::vector<std::string>* tokens, const char del) {

    (*tokens).clear();

    std::string tmp;
    for (int i = 0; i < str.length(); i++) {
        if (str[i] == del) {
            if (tmp != "") (*tokens).push_back(tmp);
            tmp = "";
        } else tmp += str[i];
    }
    if (tmp != "") (*tokens).push_back(tmp);

}

void my::assert(const bool condition, const std::string message) { if (!condition) { std::cout << "Assert check failed: " << message << "." << std::endl; exit(1); } }

void my::asserteq(const std::string value, const std::string expected, const Logger* logger) {

    if (value != expected) logger -> raise_error("Expected '%s', found '%s'.", expected.c_str(), value.c_str());

}

void my::asserteq(const int value, const int expected, const Logger* logger) {

    if (value != expected) logger -> raise_error("Expected '%d', found '%d'.", expected, value);
}

void my::assertisint(const std::string str, const Logger* logger, const int from, const int to) {

    // negative numbers accepted
    if (str[0] == '-') {
        std::string tmp = str.substr(1);
        return my::assertisint(tmp, logger);
    }

    if (str.empty() || !std::all_of(str.begin(), str.end(), ::isdigit)) logger -> raise_error("Expected a number, found '%s'.", str.c_str());

    int num = stoi(str);
    if (num < from || num > to) logger -> raise_error("Expected a number between %d and %d, found %d.", from, to, num);

}