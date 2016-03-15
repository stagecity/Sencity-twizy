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
 * @brief Binary data saved by ROS GPS module
 *
 *
 */
typedef struct {
    uint64_t stamp; /**< ROS timestamp in nano seconds */
    double latitude; /**< GPS latitude in degrees */
    double longitude; /**< GPS longitude in degrees */
    float seaHeight; /**< Height above sea in meters */
    float geoidHeight; /**< Geoidal height in meters */
    float positionDilutionOfPrecision; /**< Position dilution of precision */
    float horizontalDilutionOfPrecision; /**< Horizontal dilution of precision */
    float verticalDilutionOfPrecision; /**< Vertical dilution of precision */
    float magneticVariation; /**< Magnetic variation in degrees/s*/
    float trueCourseOverGround; /**< true course over ground in degrees */
    float magneticCourseOverGround; /**<  magnetic course over ground in degrees */
    float speedOverGround; /**< speed over ground in kilometers/hour */
    float estimatedHorizontalPosError; /**< Estimated horizontal position error in meters */
    float estimatedVerticalPosError; /**< Estimated vertical position error in meters */
    float estimatedPosError; /**< Estimated position error in meters */
    float trueEstVelocity; /**< True est velocity in meters/second */
    float trueNorthVelocity; /**< True north velocity in meters/second */
    float upVelocity; /**< Up velocity in meters/second */
    uint8_t gpsQuality; /**< GPS quality :GPS quality indication, 0 = fix not available, 1 = Non-differential GPS fix available, 2 = Differential GPS (WAAS) fix available, 6 = Estimated */
    uint8_t nbSat; /**< Number of satellite in view */
    uint8_t gsaModeAuto; /**< Automatic mode */
    uint8_t fixType; /**< Fix type, 1 = not available, 2 = 2D, 3 = 3D */
    uint8_t utcHour; /**< UTC hour */
    uint8_t utcMin; /**< UTC minute */
    uint8_t utcSec; /**< UTC second */
    uint8_t validPosition; /**< is position valid : boolean  */
    uint8_t utcDay; /**< UTC day */
    uint8_t utcMounth; /**< UTC mounth */
    uint8_t utcYear; /**< UTC Year */
    uint8_t mode;  /**< Mode indicator : A = Autonomous, D = Differential, E = Estimated, N = Data not valid */
    uint8_t prnUsed[12]; /**< PRN number, 01 to 32, of satellite used in solution, up to 12 transmitted */
} BinaryData;

void usage();

int main (int argc, char* argv[]) {
    FILE* csvFile, *binaryFile;
    char* outFileName;
    BinaryData bin;
    ssize_t binarySize = sizeof(BinaryData);    // Size of data structure

    // Check number of arg
    if(argc != 2) {
        usage();
        return EXIT_FAILURE;
    }

    // Try to open binary file in read-only
    if((binaryFile = fopen(argv[1], "r")) == NULL) {
        perror("Erreur à l'ouverture du fichier binaire");
        return EXIT_FAILURE;
    }

    // Compute output file name
    outFileName = (char*) malloc(sizeof(char) * (strlen(argv[1]) + 5));
    if(!outFileName) {
        fprintf(stderr, "Erreur mémoire\n");
        fclose(binaryFile);
        return EXIT_FAILURE;
    }

    strcpy(outFileName, argv[1]);
    strcat(outFileName, ".csv");

    // Try to open/make csv file in write-only
    // will erase previously created file
    if((csvFile = fopen(outFileName, "w")) == NULL) {
        perror("Erreur à l'ouverture du fichier csv");
        fclose(binaryFile);
        free(outFileName);
        return EXIT_FAILURE;
    }

    free(outFileName);

    // Print CSV header
    fprintf(csvFile, "Ros Timestamp,latitude,longitude,seaHeight,geoidHeight,positionDilutionOfPrecision,horizontalDilutionOfPrecision,verticalDilutionOfPrecision");
    fprintf(csvFile, ",magneticVariation,trueCourseOverGround,magneticCourseOverGround,speedOverGround,estimatedHorizontalPosError,estimatedVerticalPosError");
    fprintf(csvFile, ",estimatedPosError,trueEstVelocity,trueNorthVelocity,upVelocity,gpsQuality,nbSat,gsaModeAuto,fixType,utcHour,utcMin,utcSec");
    fprintf(csvFile, ",validPosition,utcDay,utcMounth,utcYear,mode");
    for(int i = 0; i < 12; i++)
        fprintf(csvFile, ",prn%d", i);
    fprintf(csvFile, "\n");

    // For each data structure in the file, print its values
    while(1 == fread((void*) &bin, binarySize, 1, binaryFile)) {
        fprintf(csvFile, "%lu", bin.stamp);
        fprintf(csvFile, ",%lf", bin.latitude);
        fprintf(csvFile, ",%lf", bin.longitude);
        float * fl = &bin.seaHeight;
        for (int i = 0; i < 15; i++)
            fprintf(csvFile, ",%f", fl[i]);

        uint8_t * u8 = &bin.gpsQuality;
        for (int i = 0; i < 24; i++)
            fprintf(csvFile, ",%u", u8[i]);

        fprintf(csvFile, "\n");

    }


    fclose(binaryFile);
    fclose(csvFile);
    return EXIT_SUCCESS;

}

void usage() {
    printf("Gps binary data to csv utilities\nUsage : gpsBinToCsv binaryFile\n");
}

