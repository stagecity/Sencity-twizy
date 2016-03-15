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
* @brief Char size of MAC address
*/
#define ADDRESS_SIZE    18
/**
* @brief max char size of WiFi SSID
*/
#define SSID_SIZE       30
/**
* @brief max char size of WiFi security information
*/
#define SECURITY_SIZE   50
/**
* @brief mac char size of security group
*/
#define GROUP_SIZE      20
/**
* @brief max char size of security pair information
*/
#define PAIR_SIZE       20
/**
* @brief max char size of protocol information
*/
#define PROTOCOL_SIZE   20

/**
 * @brief WiFi binary data from ROS WiFi module
 *
 *
 */
typedef struct {
    uint64_t stamp;                 /**< ROS timestamp in nano seconds */
    uint16_t frequency;             /**<  WiFi frequency (channel) in MHz */
    char address[ADDRESS_SIZE];     /**< WiFi access point MAC address */
    char ssid[SSID_SIZE];           /**< WiFi SSID */
    char security[SECURITY_SIZE];   /**< WiFi security protocol */
    char groupCipher[GROUP_SIZE];   /**< WiFi security group cipher */
    char pairwiseCiphers[PAIR_SIZE];/**< WiFi security pair ciphers */
    char protocol[PROTOCOL_SIZE];   /**< WiFi protocol */
    uint8_t signal;                 /**< signal strengh form 0 to 100(highest) */
    uint8_t encryption;             /**< is WiFi encrypted (true is non null) */
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

    // Try to open binary file in read-only
    if((binaryFile = fopen(argv[1], "r")) == NULL) {
        perror("Erreur à l'ouverture du fichier binaire");
        return EXIT_FAILURE;
    }

    // Coupute output file name
    outFileName = (char*) malloc(sizeof(char) * (strlen(argv[1]) + 5));
    if(!outFileName) {
        fprintf(stderr, "Erreur mémoire\n");
        fclose(binaryFile);
        return EXIT_FAILURE;
    }

    strcpy(outFileName, argv[1]);
    strcat(outFileName, ".csv");

    // Try to open/make output file in write-only
    if((csvFile = fopen(outFileName, "w")) == NULL) {
        perror("Erreur à l'ouverture du fichier csv");
        fclose(binaryFile);
        free(outFileName);
        return EXIT_FAILURE;
    }

    free(outFileName);

    // Print CSV header
    fprintf(csvFile, "Ros Timestamp,Address,SSID,protocol,Frequency,Signal,Encryption,security,group Cipher,pairwise Ciphers\n");


    // Print each WiFi discovered
    while(1 == fread((void*) &bin, binarySize, 1, binaryFile)) {
        bin.address[ADDRESS_SIZE - 1] = '\0';
        bin.ssid[SSID_SIZE - 1] = '\0';
        bin.security[SECURITY_SIZE - 1] = '\0';
        bin.groupCipher[GROUP_SIZE - 1] = '\0';
        bin.pairwiseCiphers[PAIR_SIZE - 1] = '\0';
        bin.protocol[PROTOCOL_SIZE - 1] = '\0';
        fprintf(csvFile, "%lu,%s,%s,%s,%u,%u,%u,%s,%s,%s\n", bin.stamp, bin.address, bin.ssid, bin.protocol, bin.frequency, bin.signal, bin.encryption, bin.security, bin.groupCipher, bin.pairwiseCiphers);

    }


    fclose(binaryFile);
    fclose(csvFile);
    return EXIT_SUCCESS;

}


void usage() {
    printf("WiFi binary data to csv utilities\nUsage : wifiBinToCsv binaryFile\n");
}

