cmake_minimum_required(VERSION 3.0)

project(virtual_client)

aux_source_directory(./code/common common)
aux_source_directory(./code/tcp tcp)
aux_source_directory(./code/uart uart)

add_executable(virtual_client code/main.cpp ${common} ${tcp} ${uart})
