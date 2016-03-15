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
* @brief Velodyne VLP16 point structure
*
*
*/
typedef struct {
    uint64_t stamp;     /**< ROS timestamp in nano seconds */
    float x;            /**< Forward (relative to car), in meter from LIDAR center */
    float y;            /**< Left (relative to car), in meter from LIDAR center */
    float z;            /**< Up (relative to car), in meter from LIDAR center */
    uint8_t ring;       /**< Velodyne laser number */
    uint8_t intensity;  /**< Laser intensity measured */
} __attribute__((packed)) Point;

void usage();

int main (int argc, char* argv[]) {
    FILE* csvFile, *binaryFile;
    char* outFileName;
    Point pts;
    ssize_t pointSize = sizeof(Point);

    // Check arg number
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

    // Try to open/make CSV file in write-only
    if((csvFile = fopen(outFileName, "w")) == NULL) {
        perror("Erreur à l'ouverture du fichier csv");
        fclose(binaryFile);
        free(outFileName);
        return EXIT_FAILURE;
    }

    free(outFileName);

    // Print CSV header
    fprintf(csvFile, "Ros Timestamp,x,y,z,intensity,ring\n");

    // Print each point
    while(1 == fread((void*) &pts, pointSize, 1, binaryFile)) {
        fprintf(csvFile, "%lu,%f,%f,%f,%u,%u\n", pts.stamp, pts.x, pts.y, pts.z, pts.intensity, pts.ring);
    }


    fclose(binaryFile);
    fclose(csvFile);
    return EXIT_SUCCESS;

}

void usage() {
    printf("Velodyne binary data to csv utilities\nUsage : binToCsv binaryFile\n");
}

