
#include <memory>

#include <spdlog/spdlog.h>
#include <spdlog/logger.h>

class ConsoleLogger
{
public:
    
    /**
     * @brief shared pointer to a ConsoleLogger
     * 
     */
    typedef std::shared_ptr<ConsoleLogger> Ptr;
    
    /**
     * @brief construct a stdout color mt called console
     * 
     */
    ConsoleLogger()
    {
        _console_logger = spdlog::stdout_color_mt("console");
    }
    
private:
    
    spdlog::logger::Ptr _console_logger;
};