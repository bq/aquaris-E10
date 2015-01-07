#include <utils/Log.h>
#include <fcntl.h>
#include <math.h>

#include "camera_custom_nvram.h"
#include "camera_custom_sensor.h"
#include "image_sensor.h"
#include "kd_imgsensor_define.h"
#include "camera_AE_PLineTable_ov5648mipiraw.h"
#include "camera_info_ov5648mipiraw.h"
#include "camera_custom_AEPlinetable.h"

const NVRAM_CAMERA_ISP_PARAM_STRUCT CAMERA_ISP_DEFAULT_VALUE =
{{
    //Version
    Version: NVRAM_CAMERA_PARA_FILE_VERSION,

    //SensorId
    SensorId: SENSOR_ID,
    ISPComm:{
        {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    	}
    },
    ISPPca: {
        #include INCLUDE_FILENAME_ISP_PCA_PARAM
    },
    ISPRegs:{
        #include INCLUDE_FILENAME_ISP_REGS_PARAM
    },
    ISPMfbMixer:{{
        {//00: MFB mixer for ISO 100
            0x00000000, 0x00000000
        },
        {//01: MFB mixer for ISO 200
            0x00000000, 0x00000000
        },
        {//02: MFB mixer for ISO 400
            0x00000000, 0x00000000
        },
        {//03: MFB mixer for ISO 800
            0x00000000, 0x00000000
        },
        {//04: MFB mixer for ISO 1600
            0x00000000, 0x00000000
        },
        {//05: MFB mixer for ISO 2400
            0x00000000, 0x00000000
        },
        {//06: MFB mixer for ISO 3200
            0x00000000, 0x00000000
        }
    }},
    ISPCcmPoly22:{
        65700,    // i4R_AVG
        12423,    // i4R_STD
        92325,    // i4B_AVG
        21844,    // i4B_STD
        {  // i4P00[9]
            4890000, -1732500, -590000, -907500, 3575000, -112500, 40000, -1895000, 4412500
        },
        {  // i4P10[9]
            873353, -947554, 72495, -106910, 109891, 6493, -30297, 203118, -168083
        },
        {  // i4P01[9]
            180003, -367558, 177125, -103500, -180477, 285129, -45950, -220470, 266996
        },
        {  // i4P20[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        {  // i4P11[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        {  // i4P02[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        }
    }
}};

const NVRAM_CAMERA_3A_STRUCT CAMERA_3A_NVRAM_DEFAULT_VALUE =
{
    NVRAM_CAMERA_3A_FILE_VERSION, // u4Version
    SENSOR_ID, // SensorId

    // AE NVRAM
    {
        // rDevicesInfo
        {
            1136,    // u4MinGain, 1024 base = 1x
            10240,    // u4MaxGain, 16x
            51,    // u4MiniISOGain, ISOxx  
            64,    // u4GainStepUnit, 1x/8 
            35,    // u4PreExpUnit 
            30,    // u4PreMaxFrameRate
            35,    // u4VideoExpUnit  
            30,    // u4VideoMaxFrameRate 
            1024,    // u4Video2PreRatio, 1024 base = 1x 
            35,    // u4CapExpUnit 
            15,    // u4CapMaxFrameRate
            1024,    // u4Cap2PreRatio, 1024 base = 1x
            24,    // u4LensFno, Fno = 2.8
            350     // u4FocusLength_100x
         },
         // rHistConfig
        {
            2,   // u4HistHighThres
            40,  // u4HistLowThres
            2,   // u4MostBrightRatio
            1,   // u4MostDarkRatio
            160, // u4CentralHighBound
            20,  // u4CentralLowBound
            {240, 230, 220, 210, 200}, // u4OverExpThres[AE_CCT_STRENGTH_NUM]
            {86, 108, 128, 148, 170},  // u4HistStretchThres[AE_CCT_STRENGTH_NUM]
            {18, 22, 26, 30, 34}       // u4BlackLightThres[AE_CCT_STRENGTH_NUM]
        },
        // rCCTConfig
        {
            TRUE,            // bEnableBlackLight
            TRUE,            // bEnableHistStretch
            FALSE,           // bEnableAntiOverExposure
            TRUE,            // bEnableTimeLPF
            TRUE,            // bEnableCaptureThres
            TRUE,            // bEnableVideoThres
            TRUE,            // bEnableStrobeThres
            54,    // u4AETarget
            50,    // u4StrobeAETarget

            50,                // u4InitIndex
            4,                 // u4BackLightWeight
            32,                // u4HistStretchWeight
            4,                 // u4AntiOverExpWeight
            2,                 // u4BlackLightStrengthIndex
            2,                 // u4HistStretchStrengthIndex
            2,                 // u4AntiOverExpStrengthIndex
            2,                 // u4TimeLPFStrengthIndex
            {1, 3, 5, 7, 8}, // u4LPFConvergeTable[AE_CCT_STRENGTH_NUM]
            90,                // u4InDoorEV = 9.0, 10 base
            -9,    // i4BVOffset delta BV = value/10 
            64,    // u4PreviewFlareOffset
            64,    // u4CaptureFlareOffset
            4,    // u4CaptureFlareThres
            64,    // u4VideoFlareOffset
            4,    // u4VideoFlareThres
            64,    // u4StrobeFlareOffset
            3,    // u4StrobeFlareThres
            160,    // u4PrvMaxFlareThres
            0,    // u4PrvMinFlareThres
            160,    // u4VideoMaxFlareThres
            0,                 // u4VideoMinFlareThres            
            18,                // u4FlatnessThres              // 10 base for flatness condition.
            75                 // u4FlatnessStrength
         }
    },

    // AWB NVRAM
    {
        // AWB calibration data
        {
            // rUnitGain (unit gain: 1.0 = 512)
            {
                0,    // i4R
                0,    // i4G
                0    // i4B
            },
            // rGoldenGain (golden sample gain: 1.0 = 512)
            {
                0,    // i4R
                0,    // i4G
                0    // i4B
            },
            // rTuningUnitGain (Tuning sample unit gain: 1.0 = 512)
            {
                0,    // i4R
                0,    // i4G
                0    // i4B
            },
            // rD65Gain (D65 WB gain: 1.0 = 512)
            {
                793,    // i4R
                512,    // i4G
                571    // i4B
            }
        },
        // Original XY coordinate of AWB light source
        {
           // Strobe
            {
                120,    // i4X
                -200    // i4Y
            },
            // Horizon
            {
                -378,    // i4X
                -243    // i4Y
            },
            // A
            {
                -285,    // i4X
                -259    // i4Y
            },
            // TL84
            {
                -127,    // i4X
                -291    // i4Y
            },
            // CWF
            {
                -103,    // i4X
                -364    // i4Y
            },
            // DNP
            {
                -57,    // i4X
                -246    // i4Y
            },
            // D65
            {
                121,    // i4X
                -202    // i4Y
            },
            // DF
            {
                -12,    // i4X
                -295    // i4Y
            }
        },
        // Rotated XY coordinate of AWB light source
        {
            // Strobe
            {
                91,    // i4X
                -215    // i4Y
            },
            // Horizon
            {
                -409,    // i4X
                -188    // i4Y
            },
            // A
            {
                -319,    // i4X
                -217    // i4Y
            },
            // TL84
            {
                -167,    // i4X
                -271    // i4Y
            },
            // CWF
            {
                -153,    // i4X
                -347    // i4Y
            },
            // DNP
            {
                -91,    // i4X
                -236    // i4Y
            },
            // D65
            {
                92,    // i4X
                -217    // i4Y
            },
            // DF
            {
                -53,    // i4X
                -291    // i4Y
            }
        },
        // AWB gain of AWB light source
        {
            // Strobe 
            {
                790,    // i4R
                512,    // i4G
                571    // i4B
            },
            // Horizon 
            {
                512,    // i4R
                615,    // i4G
                1425    // i4B
            },
            // A 
            {
                512,    // i4R
                530,    // i4G
                1106    // i4B
            },
            // TL84 
            {
                639,    // i4R
                512,    // i4G
                902    // i4B
            },
            // CWF 
            {
                729,    // i4R
                512,    // i4G
                963    // i4B
            },
            // DNP 
            {
                661,    // i4R
                512,    // i4G
                772    // i4B
            },
            // D65 
            {
                793,    // i4R
                512,    // i4G
                571    // i4B
            },
            // DF 
            {
                751,    // i4R
                512,    // i4G
                776    // i4B
            }
        },
        // Rotation matrix parameter
        {
            8,    // i4RotationAngle
            254,    // i4Cos
            36    // i4Sin
        },
        // Daylight locus parameter
        {
            -166,    // i4SlopeNumerator
            128    // i4SlopeDenominator
        },
        // AWB light area
        {
            // Strobe:FIXME
            {
            0,    // i4RightBound
            0,    // i4LeftBound
            0,    // i4UpperBound
            0    // i4LowerBound
            },
            // Tungsten
            {
            -270,    // i4RightBound
            -867,    // i4LeftBound
            -152,    // i4UpperBound
            -252    // i4LowerBound
            },
            // Warm fluorescent
            {
            -270,    // i4RightBound
            -867,    // i4LeftBound
            -252,    // i4UpperBound
            -372    // i4LowerBound
            },
            // Fluorescent
            {
            -141,    // i4RightBound
            -270,    // i4LeftBound
            -144,    // i4UpperBound
            -309    // i4LowerBound
            },
            // CWF
            {
            -141,    // i4RightBound
            -270,    // i4LeftBound
            -309,    // i4UpperBound
            -397    // i4LowerBound
            },
            // Daylight
            {
            117,    // i4RightBound
            -141,    // i4LeftBound
            -137,    // i4UpperBound
            -297    // i4LowerBound
            },
            // Shade
            {
            477,    // i4RightBound
            117,    // i4LeftBound
            -137,    // i4UpperBound
            -297    // i4LowerBound
            },
            // Daylight Fluorescent
            {
            117,    // i4RightBound
            -141,    // i4LeftBound
            -297,    // i4UpperBound
            -380    // i4LowerBound
            }
        },
        // PWB light area
        {
            // Reference area
            {
            477,    // i4RightBound
            -867,    // i4LeftBound
            0,    // i4UpperBound
            -397    // i4LowerBound
            },
            // Daylight
            {
            142,    // i4RightBound
            -141,    // i4LeftBound
            -137,    // i4UpperBound
            -297    // i4LowerBound
            },
            // Cloudy daylight
            {
            242,    // i4RightBound
            67,    // i4LeftBound
            -137,    // i4UpperBound
            -297    // i4LowerBound
            },
            // Shade
            {
            342,    // i4RightBound
            67,    // i4LeftBound
            -137,    // i4UpperBound
            -297    // i4LowerBound
            },
            // Twilight
            {
            -141,    // i4RightBound
            -301,    // i4LeftBound
            -137,    // i4UpperBound
            -297    // i4LowerBound
            },
            // Fluorescent
            {
            142,    // i4RightBound
            -267,    // i4LeftBound
            -167,    // i4UpperBound
            -397    // i4LowerBound
            },
            // Warm fluorescent
            {
            -219,    // i4RightBound
            -419,    // i4LeftBound
            -167,    // i4UpperBound
            -397    // i4LowerBound
            },
            // Incandescent
            {
            -219,    // i4RightBound
            -419,    // i4LeftBound
            -137,    // i4UpperBound
            -297    // i4LowerBound
            },
            // Gray World
            {
            5000,    // i4RightBound
            -5000,    // i4LeftBound
            5000,    // i4UpperBound
            -5000    // i4LowerBound
            }
        },
        // PWB default gain	
        {
            // Daylight
            {
            714,    // i4R
            512,    // i4G
            656    // i4B
            },
            // Cloudy daylight
            {
            852,    // i4R
            512,    // i4G
            519    // i4B
            },
            // Shade
            {
            902,    // i4R
            512,    // i4G
            481    // i4B
            },
            // Twilight
            {
            553,    // i4R
            512,    // i4G
            921    // i4B
            },
            // Fluorescent
            {
            733,    // i4R
            512,    // i4G
            779    // i4B
            },
            // Warm fluorescent
            {
            546,    // i4R
            512,    // i4G
            1152    // i4B
            },
            // Incandescent
            {
            494,    // i4R
            512,    // i4G
            1069    // i4B
            },
            // Gray World
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            }
        },
        // AWB preference color	
        {
            // Tungsten
            {
            0,    // i4SliderValue
            6102    // i4OffsetThr
            },
            // Warm fluorescent	
            {
            0,    // i4SliderValue
            5978    // i4OffsetThr
            },
            // Shade
            {
            0,    // i4SliderValue
            1344    // i4OffsetThr
            },
            // Daylight WB gain
            {
            642,    // i4R
            512,    // i4G
            755    // i4B
            },
            // Preference gain: strobe
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: tungsten
            {
            506,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: warm fluorescent
            {
            504,    // i4R
            512,    // i4G
            500    // i4B
            },
            // Preference gain: fluorescent
            {
            504,    // i4R
            512,    // i4G
            520    // i4B
            },
            // Preference gain: CWF
            {
            512,    // i4R
            512,    // i4G
            520    // i4B
            },
            // Preference gain: daylight
            {
            512,    // i4R
            512,    // i4G
            515    // i4B
            },
            // Preference gain: shade
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: daylight fluorescent
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            }
        },
        {// CCT estimation
            {// CCT
                2300,    // i4CCT[0]
                2850,    // i4CCT[1]
                4100,    // i4CCT[2]
                5100,    // i4CCT[3]
                6500    // i4CCT[4]
            },
            {// Rotated X coordinate
                -501,    // i4RotatedXCoordinate[0]
                -411,    // i4RotatedXCoordinate[1]
                -259,    // i4RotatedXCoordinate[2]
                -183,    // i4RotatedXCoordinate[3]
                0    // i4RotatedXCoordinate[4]
            }
        }
    },
    {0}
};

#include INCLUDE_FILENAME_ISP_LSC_PARAM
//};  //  namespace


typedef NSFeature::RAWSensorInfo<SENSOR_ID> SensorInfoSingleton_T;


namespace NSFeature {
template <>
UINT32
SensorInfoSingleton_T::
impGetDefaultData(CAMERA_DATA_TYPE_ENUM const CameraDataType, VOID*const pDataBuf, UINT32 const size) const
{
    UINT32 dataSize[CAMERA_DATA_TYPE_NUM] = {sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT),
                                             sizeof(NVRAM_CAMERA_3A_STRUCT),
                                             sizeof(NVRAM_CAMERA_SHADING_STRUCT),
                                             sizeof(NVRAM_LENS_PARA_STRUCT),
                                             sizeof(AE_PLINETABLE_T)};

    if (CameraDataType > CAMERA_DATA_AE_PLINETABLE || NULL == pDataBuf || (size < dataSize[CameraDataType]))
    {
        return 1;
    }

    switch(CameraDataType)
    {
        case CAMERA_NVRAM_DATA_ISP:
            memcpy(pDataBuf,&CAMERA_ISP_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT));
            break;
        case CAMERA_NVRAM_DATA_3A:
            memcpy(pDataBuf,&CAMERA_3A_NVRAM_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_3A_STRUCT));
            break;
        case CAMERA_NVRAM_DATA_SHADING:
            memcpy(pDataBuf,&CAMERA_SHADING_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_SHADING_STRUCT));
            break;
        case CAMERA_DATA_AE_PLINETABLE:
            memcpy(pDataBuf,&g_PlineTableMapping,sizeof(AE_PLINETABLE_T));
            break;
        default:
            break;
    }
    return 0;
}};  //  NSFeature


