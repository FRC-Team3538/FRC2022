#include <ctre/phoenix.h>
#include <iostream>

class FalconHelper {
public:
    static void CheckError(ctre::phoenix::ErrorCode error) {
        if (error == ctre::phoenix::ErrorCode::SigNotUpdated) {
            std::cout << "CTRE Device Error: Sig Not Updated" << std::endl;
        }
        else if (error != ctre::phoenix::ErrorCode::OK) {
            std::cout << "CTRE Device Error: " << error << std::endl;
            assert(error == ctre::phoenix::ErrorCode::OK);
        }
    }
};