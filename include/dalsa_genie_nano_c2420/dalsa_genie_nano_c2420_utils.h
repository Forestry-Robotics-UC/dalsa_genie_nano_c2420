#include "gevapi.h"	
#include "dalsa_genie_nano_c2420/X_Display_utils.h"

#define MAX_CAMERAS		         256
#define ENABLE_BAYER_CONVERSION  1
#define NUM_BUF	                 8

typedef struct tagMY_CONTEXT
{
    X_VIEW_HANDLE     View;
	GEV_CAMERA_HANDLE camHandle;
	int				depth;
	int 			format;
	void 			*convertBuffer;
	BOOL			convertFormat;
	BOOL            exit;
}MY_CONTEXT, *PMY_CONTEXT;

int IsTurboDriveAvailable(GEV_CAMERA_HANDLE handle);