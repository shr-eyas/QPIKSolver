#include <stdio.h>
#include <stdlib.h>
#include "eigen3/Eigen/Dense"
#include <iostream>
#include <fstream>
#include <math.h>
#include <algorithm>
#include <time.h>

using namespace Eigen;
using namespace std;

int main() {
    // Declare an array of 7 Vector3d objects
    Vector3d Link_pos[7];

    // Assign values to the first two positions of the array
    Link_pos[0] = Vector3d(1.0, 2.0, 3.0);  // Position of the 1st link
    Link_pos[1] = Vector3d(2.0, 3.0, 4.0);  // Position of the 2nd link

    // Output the positions to verify
    cout << "Link 0 Position: " << Link_pos[0] << endl;
    cout << "Link 1 Position: " << Link_pos[1].transpose() << endl;

    return 0;
}