/*
 * Copyright (C) 2015  TELECOM Nancy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

/**
* @brief IMU binary data from ROS IMU module
*
*
*/
typedef struct {
    uint64_t stamp; /**< ROS timestamp in nanoseconds */
    double accel[3]; /**< 3D accelerometer vector. Composant in meter/second^2 (SI); 0:x - 1:y - 2:z  */
    double gyro[3]; /**< 3D gyroscope vector. Composant in rad/second; 0:x - 1:y - 2:z  */
    double mag[3]; /**< 3D magnetometer vector. Composant in Tesla; 0:x - 1:y - 2:z  */
} BinaryData;

void usage();

int main (int argc, char* argv[]) {
    FILE* csvFile, *binaryFile;
    char* outFileName;
    BinaryData bin;
    ssize_t binarySize = sizeof(BinaryData);

    // Check number of arg
    if(argc != 2) {
        usage();
        return EXIT_FAILURE;
    }

    // Try to open binary fil in read-only
    if((binaryFile = fopen(argv[1], "r")) == NULL) {
        perror("Erreur à l'ouverture du fichier binaire");
        return EXIT_FAILURE;
    }

    // Compute output filename
    outFileName = (char*) malloc(sizeof(char) * (strlen(argv[1]) + 5));
    if(!outFileName) {
        fprintf(stderr, "Erreur mémoire\n");
        fclose(binaryFile);
        return EXIT_FAILURE;
    }

    strcpy(outFileName, argv[1]);
    strcat(outFileName, ".csv");

    // Try to open/make CSV file in write only
    if((csvFile = fopen(outFileName, "w")) == NULL) {
        perror("Erreur à l'ouverture du fichier csv");
        fclose(binaryFile);
        free(outFileName);
        return EXIT_FAILURE;
    }

    free(outFileName);

    // Print CSV header
    fprintf(csvFile, "Ros Timestamp,Accel X,Accel Y,Accel Z,Gyro X,Gyro Y,Gyro Z,Mag X,Mag Y,Mag Z\n");


    // read each data structure and print it
    while(1 == fread((void*) &bin, binarySize, 1, binaryFile)) {
        fprintf(csvFile, "%lu", bin.stamp);
        double * lf = &bin.accel[0];
        for (int i = 0; i < 9; i++)
            fprintf(csvFile, ",%lf", lf[i]);

        fprintf(csvFile, "\n");

    }


    fclose(binaryFile);
    fclose(csvFile);
    return EXIT_SUCCESS;

}

void usage() {
    printf("IMU binary data to csv utilities\nUsage : imuBinToCsv binaryFile\n");
}

