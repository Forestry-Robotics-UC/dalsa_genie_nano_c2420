#include "dalsa_genie_nano_c2420/dalsa_genie_nano_c2420_utils.h"

int IsTurboDriveAvailable(GEV_CAMERA_HANDLE handle)
{
	int type;
	UINT32 val = 0;
	
	if ( 0 == GevGetFeatureValue( handle, "transferTurboCurrentlyAbailable",  &type, sizeof(UINT32), &val) )
	{
		// Current / Standard method present - this feature indicates if TurboMode is available.
		// (Yes - it is spelled that odd way on purpose).
		return (val != 0);
	}
	else
	{
		// Legacy mode check - standard feature is not there try it manually.
		char pxlfmt_str[64] = {0};

		// Mandatory feature (always present).
		GevGetFeatureValueAsString( handle, "PixelFormat", &type, sizeof(pxlfmt_str), pxlfmt_str);

		// Set the "turbo" capability selector for this format.
		if ( 0 != GevSetFeatureValueAsString( handle, "transferTurboCapabilitySelector", pxlfmt_str) )
		{
			// Either the capability selector is not present or the pixel format is not part of the 
			// capability set.
			// Either way - TurboMode is NOT AVAILABLE.....
			return 0; 
		}
		else
		{
			// The capabilty set exists so TurboMode is AVAILABLE.
			// It is up to the camera to send TurboMode data if it can - so we let it.
			return 1;
		}
	}
	return 0;
}