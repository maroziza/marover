find_package(PkgConfig REQUIRED)
pkg_check_modules(gpiodcxx REQUIRED IMPORTED_TARGET libgpiodcxx)

add_executable (ads1115_printer Ads1115Printer.cpp)
target_link_libraries(ads1115_printer
    device_kit
    PkgConfig::gpiodcxx
)
