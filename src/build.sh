cd ..
g++  -Wall  -c  -pthread -lrt  "src/main.cpp" "src/serial.cpp" "src/util.cpp" 
g++  -o ems_logger_2 main.o serial.o util.o -pthread -lrt
rm main.o serial.o util.o 
