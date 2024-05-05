#include <stdio.h>
#include <stdbool.h>
#define AMOUNT_OF_TARGETS 9

void save_targets_data(short targets_array[AMOUNT_OF_TARGETS][3], const char *filename)
{
    // Open file for writing
    FILE *file = fopen(filename, "w");
    if (!file)
    {
        printf("Error: Unable to open file for writing\n");
        return;
    }
    // Write data to file
    for (int i = 0; i < AMOUNT_OF_TARGETS; i++)
    {
        fprintf(file, "%d,%d,%d\n", targets_array[i][0], targets_array[i][1], targets_array[i][2]);
    }
    // Close file
    fclose(file);
    printf("Data saved to file: %s\n", filename);
}