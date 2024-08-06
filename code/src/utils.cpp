#include "../include/utils.hpp"

HPLUS_Environment HPLUS_env;
HPLUS_Statistics HPLUS_stats;

// ##################################################################### //
// ############################## BITFIELD ############################# //
// ##################################################################### //

my::BitField::BitField(unsigned int size) { this -> size_ = size; this -> field_ = std::vector<char>((size+7)/8, 0); }

my::BitField::BitField(const BitField& bf) { this -> size_ = bf.size_; this -> field_ = std::vector<char>(bf.field_); }

void my::BitField::set(const unsigned int i) {
    
    #if HPLUS_INTCHECK
    MYASSERT(i < this -> size_);
    #endif
    this -> field_[i/8] |= (1 << i%8);
    
}

void my::BitField::unset(const unsigned int i) {
    
    #if HPLUS_INTCHECK
    MYASSERT(i < this -> size_);
    #endif
    this -> field_[i/8] &= ~(1 << i%8);
    
}

bool my::BitField::operator[](const unsigned int i) const {
    
    #if HPLUS_INTCHECK
    MYASSERT(i < this -> size_);
    #endif
    return this -> field_[i/8] & (1 << i%8);
    
}

void my::BitField::intersect(const BitField& bf) {

    #if HPLUS_INTCHECK
    MYASSERT(this -> size_ == bf.size_);
    #endif
    for (auto i : *this) {
        if (bf[i]) this -> set(i);
        else this -> unset(i);
    }

}

void my::BitField::unificate(const BitField& bf) {

    #if HPLUS_INTCHECK
    MYASSERT(this -> size_ == bf.size_);
    #endif
    for (auto i : bf) this -> set(i);

}

bool my::BitField::equals(const BitField& bf) const {

    #if HPLUS_INTCHECK
    MYASSERT(this -> size_ == bf.size_);
    #endif
    unsigned int i;
    for (i = 0; i < this -> field_.size() && this -> field_[i] == bf.field_[i]; i++) {}
    return i == this -> field_.size();

}

bool my::BitField::contains(const BitField& bf) const {

    for (auto i : bf) if (!this -> operator[](i)) return false;
    return true;

}


unsigned int my::BitField::size() const { return this -> size_; }

std::string my::BitField::view() const {

    std::string ret;
    for (unsigned int i = 0; i < this -> size_; i++) ret.append(std::to_string(this -> operator[](i)));
    return ret;

}

// ##################################################################### //
// ############################### LOGGER ############################## //
// ##################################################################### //

void my::Logger::format_output_(const char* str, va_list ptr, FILE* log_file, const bool show_time) {

    if (show_time) {
        double elapsed_time = HPLUS_env.get_time();
        fprintf(stdout, "%8.3f : ", elapsed_time);
        if (HPLUS_env.log) fprintf(log_file, "%8.3f : ", elapsed_time);
    }

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
                if (HPLUS_env.log) fprintf(log_file, "%s", token);

            } else { 

                int j = 1; 
                char ch1;

                // this loop is required when printing formatted value like 0.2f, when ch1='f' loop ends 
                while ((ch1 = token[j++]) < 58) {}

                if (ch1 == 'i' || ch1 == 'd' || ch1 == 'u' || ch1 == 'h') { // for integers

                    int value = va_arg(ptr, int);
                    fprintf(stdout, token, value);
                    if (HPLUS_env.log) fprintf(log_file, token, value);

                } else if (ch1 == 'c') { // for chars

                    char value = va_arg(ptr, int);
                    fprintf(stdout, token, value);
                    if (HPLUS_env.log) fprintf(log_file, token, value);

                } else if (ch1 == 'f') { // for float values 

                    double value = va_arg(ptr, double);
                    fprintf(stdout, token, value);
                    if (HPLUS_env.log) fprintf(log_file, token, value);

                } else if (ch1 == 'l') { // for long values

                    char ch2 = token[2];

                    if (ch2 == 'u' || ch2 == 'd' || ch2 == 'i') { // for long int values 

                        long value = va_arg(ptr, long);
                        fprintf(stdout, token, value);
                        if (HPLUS_env.log) fprintf(log_file, token, value);

                    } else if (ch2 == 'f') {  // for double values 

                        double value = va_arg(ptr, double);
                        fprintf(stdout, token, value);
                        if (HPLUS_env.log) fprintf(log_file, token, value);

                    } 

                } else if (ch1 == 'L') { 

                    char ch2 = token[2]; 

                    if (ch2 == 'u' || ch2 == 'd' || ch2 == 'i') { // for long long int values 

                        long long value = va_arg(ptr, long long);
                        fprintf(stdout, token, value);
                        if (HPLUS_env.log) fprintf(log_file, token, value);

                    } 

                    else if (ch2 == 'f') { // for long double values 

                        long double value = va_arg(ptr, long double);
                        fprintf(stdout, token, value);
                        if (HPLUS_env.log) fprintf(log_file, token, value);

                    } 

                } else if (ch1 == 's') { // for string values 

                    char* value = va_arg(ptr, char*);
                    fprintf(stdout, token, value);
                    if (HPLUS_env.log) fprintf(log_file, token, value);

                } else { // print the whole token if no case is matched 

                    fprintf(stdout, "%s", token);
                    if (HPLUS_env.log) fprintf(log_file, "%s", token);

                } 

            } 
        
        }

    }
}

my::Logger::Logger(const std::string& run_title, const std::string& log_name) {

    this -> log_file_name_ = log_name;
    if (HPLUS_env.log)  {
        FILE* log_file = fopen(this -> log_file_name_.c_str(), "a");
        fprintf(log_file, "\n%s\nRUN_NAME: %s\n%s\n", THICK_LINE, run_title.c_str(), THICK_LINE);
        fclose(log_file);
    }

}

void my::Logger::print(const char* str, ...) const {

    #if HPLUS_VERBOSE == 0
    return;
    #endif

    // logging file
    FILE* log_file = nullptr;
    if (HPLUS_env.log) log_file = fopen(this -> log_file_name_.c_str(), "a");

    // initializing list pointer
    va_list ptr;
    va_start(ptr, str);

    // printing and logging the formatted message
    format_output_(str, ptr, log_file, false);

    fprintf(stdout, "\n");
    if (HPLUS_env.log) fprintf(log_file, "\n");

    if (HPLUS_env.log) fclose(log_file);

    // ending traversal
    va_end(ptr);

}

void my::Logger::print_info(const char* str, ...) const {

    #if HPLUS_VERBOSE < 10
    return;
    #endif

    // logging file
    FILE* log_file = nullptr;
    if (HPLUS_env.log) log_file = fopen(this -> log_file_name_.c_str(), "a");

    // initializing list pointer 
    va_list ptr; 
    va_start(ptr, str);

    fprintf(stdout, "\033[92m\033[1m[ INFO  ]\033[0m -- ");
    if (HPLUS_env.log) fprintf(log_file, "[ INFO  ] -- ");

    // printing and logging the formatted message
    format_output_(str, ptr, log_file);

    fprintf(stdout, "\n");
    if (HPLUS_env.log) fprintf(log_file, "\n");
    
    if (HPLUS_env.log) fclose(log_file);

    // ending traversal 
    va_end(ptr);

}

void my::Logger::print_warn(const char* str, ...) const {

    #if HPLUS_WARN == 0
    return;
    #endif

    // logging file
    FILE* log_file = nullptr;
    if (HPLUS_env.log) log_file = fopen(this -> log_file_name_.c_str(), "a");

    // initializing list pointer 
    va_list ptr; 
    va_start(ptr, str);

    fprintf(stdout, "\033[93m\033[1m[ WARN  ]\033[0m -- ");
    if (HPLUS_env.log) fprintf(log_file, "[ WARN  ] -- ");

    // printing and logging the formatted message
    format_output_(str, ptr, log_file);

    fprintf(stdout, "\n");
    if (HPLUS_env.log) fprintf(log_file, "\n");
    
    if (HPLUS_env.log) fclose(log_file);

    // ending traversal 
    va_end(ptr);

}

void my::Logger::raise_error(const char* str, ...) const {

    // logging file
    FILE* log_file = nullptr;
    if (HPLUS_env.log) log_file = fopen(this -> log_file_name_.c_str(), "a");

    // initializing list pointer 
    va_list ptr; 
    va_start(ptr, str);

    fprintf(stdout, "\033[91m\033[1m[ ERROR ]\033[0m -- ");
    if (HPLUS_env.log) fprintf(log_file, "[ ERROR ] -- ");

    // printing and logging the formatted message
    format_output_(str, ptr, log_file);

    fprintf(stdout, "\n");
    if (HPLUS_env.log) fprintf(log_file, "\n");
    
    if (HPLUS_env.log) fclose(log_file);

    // ending traversal 
    va_end(ptr);

    exit(1);

}

// ##################################################################### //
// ############################## GLOBALS ############################## //
// ##################################################################### //

bool HPLUS_Environment::found() const { return this -> status < my::status::INFEAS; }

void HPLUS_Environment::start_timer() { this -> timer = std::chrono::steady_clock::now(); }
double HPLUS_Environment::get_time() const { return ((double) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - this -> timer).count()) / 1000; }

void HPLUS_Statistics::print() const {

    HPLUS_env.logger.print("\n\n------------------------------------------");
    HPLUS_env.logger.print("-------------   Statistics   -------------");
    HPLUS_env.logger.print("------------------------------------------\n");
    HPLUS_env.logger.print(" >>  Parsing time         %10.3fs  <<", this->parsing_time);
    HPLUS_env.logger.print(" >>  Build time           %10.3fs  <<", this->build_time);
    HPLUS_env.logger.print(" >>  Exec time            %10.3fs  <<", this->exec_time);
    HPLUS_env.logger.print("\n\n");

}

// ##################################################################### //
// ############################ MY NAMESPACE ########################### //
// ##################################################################### //

void my::split(const std::string& str, std::vector<std::string>* tokens, const char del) {

    tokens -> clear();

    std::string tmp;
    for (int i = 0; i < str.length(); i++) {
        if (str[i] == del) {
            if (!tmp.empty()) tokens -> push_back(tmp);
            tmp = "";
        } else tmp += str[i];
    }
    if (!tmp.empty()) tokens -> push_back(tmp);

}

void my::assert(const bool condition, const std::string& message) { if (!condition) HPLUS_env.logger.raise_error("%s - Assert check failed.", message.c_str()); }

bool my::isint(const std::string& str, const int from, const int to) {

    try {
        int num = stoi(str);
        return num >= from && num <= to;
    } catch (std::invalid_argument&) { return false; }

}