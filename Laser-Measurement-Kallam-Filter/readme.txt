The Kalman Filter to do the laser measurement 

Follow the following step

1. sudo apt-get update
2. sudo apt-get install git
3. sudo apt-get install cmake
4. sudo apt-get install openssl
5. sudo apt-get install libssl-dev
6. git clone https://github.com/jwangjie/SDC-Extended-Kalman-Filter-Project/tree/master/Laser-Measurement-Kallam-Filter
7. cd ../Laser-Measurement-Kallam-Filter
8. mkdir build && cd build
9. cmake .. && make
10. ./ExtendedKF ../data/obj_pose-laser-radar-synthetic-input.txt output.txt
