/*--------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>


#include "GeomagnetismHeader.h"
#include "GeomagnetismLibrary.c"
#include "EGM9615.h"

/*---------------------------------------------------------------------------*/

/* 
WMM Point Calculation Program.

The Geomagnetism Library is used to make a command prompt program. The program prompts
the user to enter a location, performs the computations and prints the results to the
standard output. The program expects the files GeomagnetismLibrary.c, GeomagnetismHeader.h,
WMM.COF and EGM9615.h to be in the same directory. 

Manoj.C.Nair@Noaa.Gov
April 21, 2011

 */
char MAG_GeomagIntroduction_WMM(MAGtype_MagneticModel *MagneticModel, char *VersionDate, char *ModelDate);


int main(int argc, char* argv[])
{
    MAGtype_MagneticModel * MagneticModels[1], *TimedMagneticModel;
    MAGtype_Ellipsoid Ellip;
    MAGtype_CoordSpherical CoordSpherical;
    MAGtype_CoordGeodetic CoordGeodetic;
    MAGtype_Date UserDate;
    MAGtype_GeoMagneticElements GeoMagneticElements, Errors;
    MAGtype_Geoid Geoid;
    char ans[20];
    char filename[] = "WMM.COF";
    char VersionDate[12];
    int NumTerms, nMax = 0;
    int epochs = 1;

    if (argc != 5) printf("4 Arguments needed\n");

    /* Memory allocation */

    strncpy(VersionDate, VERSIONDATE_LARGE + 39, 11);
    VersionDate[11] = '\0';
    if(!MAG_robustReadMagModels(filename, &MagneticModels, epochs)) {
        printf("\n WMM.COF not found.  Press enter to exit... \n ");
        fgets(ans, 20, stdin);
        return 1;
    }
    if(nMax < MagneticModels[0]->nMax) nMax = MagneticModels[0]->nMax;
    NumTerms = ((nMax + 1) * (nMax + 2) / 2);
    TimedMagneticModel = MAG_AllocateModelMemory(NumTerms); /* For storing the time modified WMM Model parameters */
    if(MagneticModels[0] == NULL || TimedMagneticModel == NULL)
    {
        MAG_Error(2);
    }
    MAG_SetDefaults(&Ellip, &Geoid); /* Set default values and constants */
    /* Check for Geographic Poles */



    /* Set EGM96 Geoid parameters */
    Geoid.GeoidHeightBuffer = GeoidHeightBuffer;
    Geoid.Geoid_Initialized = 1;
    /* Set EGM96 Geoid parameters END */

    /* Recovery of the arguments */
    sscanf(argv[1], "%lf", &CoordGeodetic.phi); //Please enter latitude : North latitude positive, For example: 30.508 (Decimal Degrees)
    sscanf(argv[2], "%lf", &CoordGeodetic.lambda); //Please enter longitude : East longitude positive, West negative.  For example: -100.5 for 100.5 degrees west
    Geoid.UseGeoid = 1;
    sscanf(argv[3], "%lf", &CoordGeodetic.HeightAboveGeoid); //Please enter height above mean sea level (in kilometers)]
    sscanf(argv[4], "%lf", &UserDate.DecimalYear); //Please enter the decimal year (YYYY.yyy)
    
    MAG_ConvertGeoidToEllipsoidHeight(&CoordGeodetic, &Geoid);
    MAG_GeodeticToSpherical(Ellip, CoordGeodetic, &CoordSpherical); /*Convert from geodetic to Spherical Equations: 17-18, WMM Technical report*/
    MAG_TimelyModifyMagneticModel(UserDate, MagneticModels[0], TimedMagneticModel); /* Time adjust the coefficients, Equation 19, WMM Technical report */
    MAG_Geomag(Ellip, CoordSpherical, CoordGeodetic, TimedMagneticModel, &GeoMagneticElements); /* Computes the geoMagnetic field elements and their time change*/
    MAG_CalculateGridVariation(CoordGeodetic, &GeoMagneticElements);
    MAG_WMMErrorCalc(GeoMagneticElements.H, &Errors);
    MAG_PrintData(GeoMagneticElements, TimedMagneticModel); /* Print the results */
  
    MAG_FreeMagneticModelMemory(TimedMagneticModel);
    MAG_FreeMagneticModelMemory(MagneticModels[0]);

    return 0;
}

