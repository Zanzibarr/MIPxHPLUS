#include "../include/utils.hpp"

HPLUS_Environment HPLUS_env;
HPLUS_Statistics HPLUS_stats;

// ##################################################################### //
// ############################## BITFIELD ############################# //
// ##################################################################### //

my::BitField::BitField(unsigned int size) {

    #if INTCHECKS
    ASSERT(size > 0);
    #endif
    this -> len = size; this -> field = new char[(size+7)/8]();

}

my::BitField::~BitField() { MYDELL(this->field); }

void my::BitField::set(const unsigned int i) {
    
    #if INTCHECKS
    ASSERT(i < this -> len);
    #endif
    this -> field[i/8] |= (1 << i%8);
    
}

void my::BitField::unset(const unsigned int i) {
    
    #if INTCHECKS
    ASSERT(i < this -> len);
    #endif
    this -> field[i/8] &= ~(1 << i%8);
    
}

bool my::BitField::operator[](const unsigned int i) const {
    
    #if INTCHECKS
    ASSERT(i < this -> len);
    #endif
    return this -> field[i/8] & (1 << i%8);
    
}

my::BitField my::BitField::operator&(const BitField bf) const {

    #if INTCHECKS
    ASSERT(this -> len == bf.len);
    #endif
    BitField ret = BitField(this -> len);
    for (int i = 0; i < this -> len; i++) if ((this -> field[i/8] & (1 << i%8)) && bf[i]) ret.set(i);
    return ret;

}

bool my::BitField::operator==(const BitField bf) const {

    #if INTCHECKS
    ASSERT(this -> len == bf.len);
    #endif
    for (int i = 0; i < (this -> len + 7) / 8; i++) if (this -> field[i] != bf.field[i]) return false;
    return true;

}

unsigned int my::BitField::size() const { return this -> len; }

std::string my::BitField::view() const {

    std::string ret;
    for (int i = 0; i < this -> len; i++) ret.append(std::to_string(this -> operator[](i)));
    return ret;

}

// ##################################################################### //
// ############################### LOGGER ############################## //
// ##################################################################### //

void my::Logger::_format_output(const char* str, va_list ptr, FILE* log_file, const bool show_time) const {

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
                char ch1 = 0; 

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

my::Logger::Logger(const std::string run_title, const std::string log_name) {

    this -> log_file_name = log_name;
    if (HPLUS_env.log)  {
        FILE* log_file = fopen(this -> log_file_name.c_str(), "a");
        fprintf(log_file, "\n%s\nRUN_NAME: %s\n%s\n", THICK_LINE, run_title.c_str(), THICK_LINE);
        fclose(log_file);
    }

}

void my::Logger::print(const char* str, ...) const {

    // logging file
    FILE* log_file = fopen(this -> log_file_name.c_str(), "a");

    // initializing list pointer
    va_list ptr;
    va_start(ptr, str);

    // printing and logging the formatted message
    _format_output(str, ptr, log_file, false);

    fprintf(stdout, "\n");
    if (HPLUS_env.log) fprintf(log_file, "\n");

    fclose(log_file);

    // ending traversal
    va_end(ptr);

}

void my::Logger::print_info(const char* str, ...) const {

    #if HPLUS_VERBOSE < 10
    return;
    #endif

    // logging file
    FILE* log_file = fopen(this -> log_file_name.c_str(), "a");

    // initializing list pointer 
    va_list ptr; 
    va_start(ptr, str);

    fprintf(stdout, "\033[92m\033[1m[ INFO  ]\033[0m -- ");
    if (HPLUS_env.log) fprintf(log_file, "[ INFO  ] -- ");

    // printing and logging the formatted message
    _format_output(str, ptr, log_file);

    fprintf(stdout, "\n");
    if (HPLUS_env.log) fprintf(log_file, "\n");
    
    fclose(log_file);

    // ending traversal 
    va_end(ptr);

}

void my::Logger::print_warn(const char* str, ...) const {

    #if HPLUS_WARN == 0
    return;
    #endif

    // logging file
    FILE* log_file = fopen(this -> log_file_name.c_str(), "a");

    // initializing list pointer 
    va_list ptr; 
    va_start(ptr, str);

    fprintf(stdout, "\033[93m\033[1m[ WARN  ]\033[0m -- ");
    if (HPLUS_env.log) fprintf(log_file, "[ WARN  ] -- ");

    // printing and logging the formatted message
    _format_output(str, ptr, log_file);

    fprintf(stdout, "\n");
    if (HPLUS_env.log) fprintf(log_file, "\n");
    
    fclose(log_file);

    // ending traversal 
    va_end(ptr);

}

void my::Logger::raise_error(const char* str, ...) const {

    // logging file
    FILE* log_file = fopen(this -> log_file_name.c_str(), "a");

    // initializing list pointer 
    va_list ptr; 
    va_start(ptr, str);

    fprintf(stdout, "\033[91m\033[1m[ ERROR ]\033[0m -- ");
    if (HPLUS_env.log) fprintf(log_file, "[ ERROR ] -- ");

    // printing and logging the formatted message
    _format_output(str, ptr, log_file);

    fprintf(stdout, "\n");
    if (HPLUS_env.log) fprintf(log_file, "\n");
    
    fclose(log_file);

    // ending traversal 
    va_end(ptr);

    exit(1);

}

// ##################################################################### //
// ############################## GLOBALS ############################## //
// ##################################################################### //

unsigned int HPLUS_Environment::cpx_act_idx(unsigned int act_i) { return HPLUS_env.act_start + act_i; }
unsigned int HPLUS_Environment::cpx_tact_idx(unsigned int tact_i) { return HPLUS_env.tact_start + tact_i; }
unsigned int HPLUS_Environment::cpx_var_idx(unsigned int var_i) { return HPLUS_env.var_start + var_i; }
unsigned int HPLUS_Environment::cpx_fa_idx(unsigned int fa_i) { return HPLUS_env.fa_start + fa_i; }
unsigned int HPLUS_Environment::cpx_tvar_idx(unsigned int t_var_i) { return HPLUS_env.tvar_start + t_var_i; }

void HPLUS_Environment::start_timer() { timer = std::chrono::steady_clock::now(); }

double HPLUS_Environment::get_time() { return ((double) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timer).count()) / 1000; }

void HPLUS_Statistics::print() const {

    HPLUS_env.logger.print("\n\n------------------------------------------");
    HPLUS_env.logger.print("-------------   Statistics   -------------");
    HPLUS_env.logger.print("------------------------------------------\n");
    HPLUS_env.logger.print(" >>  Parsing time         %10.3fs  <<", this->parsing_time);
    HPLUS_env.logger.print(" >>  Exec time            %10.3fs  <<", this->exec_time);
    HPLUS_env.logger.print("\n\n");

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

void my::assert(const bool condition, const std::string message) { if (!condition) HPLUS_env.logger.raise_error("%s - Assert check failed.", message.c_str()); }

bool my::isint(const std::string str, const int from, const int to) {

    try {
        int num = stoi(str);
        return num >= from && num <= to;
    } catch (std::invalid_argument) { return false; }

}