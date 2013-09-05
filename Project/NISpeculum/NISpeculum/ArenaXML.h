#ifndef _ARENA_XML
#define _ARENA_XML

#include <ToolBoxXML.h>

#define _XML_FLAG "flag"
#define _XML_VALUE "value"
#define _XML_PATH "path"

//ARENA

//ARENA FLAGS
#define _XML_ARENA_ELEM_FLAGS "Flags"
	#define _XML_ARENA_ELEM_FLAG_FLOOR "Floor"
	#define _XML_ARENA_ELEM_FLAG_MIRRORS "Mirrors"

#define _XML_ARENA_ELEM_FLOOR "Floor"

#define _XML_ARENA_ELEM_MIRRORS "Mirrors"
	#define _XML_ARENA_ATT_MIRRORS_N "n_mirrors"
	#define _XML_ARENA_ELEM_MIRROR "Mirror"

//FLOOR

//FLOOR FLAGS
#define _XML_FLOOR_FILE_AREA_MASK "floor_area_mask.png"
#define _XML_FLOOR_FILE_FLOOR_MASK "floor_mask.png"

#define _XML_FLOOR_ELEM_FLAGS "Flags"
	#define _XML_FLOOR_ELEM_FLAG_PLANE "Plane"
	#define _XML_FLOOR_ELEM_FLAG_AREA "Area"
	#define _XML_FLOOR_ELEM_FLAG_FLOOR_MASK "Mask"
	#define _XML_FLOOR_ELEM_FLAG_INPUT "Input"

//Floor Area
#define _XML_FLOOR_ELEM_AREA "Area"
	#define _XML_FLOOR_ELEM_AREA_MASK "AreaMask"
		//_XML_PATH
	#define _XML_FLOOR_ELEM_AREA_LIMITS "Limits"
		#define _XML_FLOOR_ELEM_AREA_MAX_WIDTH "MaxWidth"
			//_XML_VALUE
		#define _XML_FLOOR_ELEM_AREA_MIN_WIDTH "MinWidth"
			//_XML_VALUE
		#define _XML_FLOOR_ELEM_AREA_MAX_HEIGHT "MaxHeight"
			//_XML_VALUE
		#define _XML_FLOOR_ELEM_AREA_MIN_HEIGHT "MinHeight"
			//_XML_VALUE

//Floor Plane
#define _XML_FLOOR_ELEM_PLANE "Plane"
	#define _XML_FLOOR_ELEM_PLANE_A "A"
		//_XML_VALUE
	#define _XML_FLOOR_ELEM_PLANE_B "B"
		//_XML_VALUE
	#define _XML_FLOOR_ELEM_PLANE_C "C"
		//_XML_VALUE
	#define _XML_FLOOR_ELEM_PLANE_D "D"
		//_XML_VALUE

//Floor Mask
#define _XML_FLOOR_ELEM_MASK "Mask"
	#define _XML_FLOOR_ELEM_MASK_MASK "FloorMask"
		//_XML_PATH
	#define _XML_FLOOR_ELEM_MASK_THRESH "Threshold"
		//_XML_VALUE

//Floor Input
#define _XML_FLOOR_ELEM_INPUT "Input"
	#define _XML_FLOOR_ELEM_INPUT_N "NPoints"
		//_XML_VALUE
	#define _XML_FLOOR_ELEM_INPUT_POINTS "Points"
		#define _XML_FLOOR_ELEM_INPUT_POINT "Point"
		//CV::POINT


//MIRROR
#define _XML_MIRROR_FILE_AREA_MASK "mirror_area_mask_"
#define _XML_MIRROR_FILE_AREA_MASK_PNG ".png"

//MIRROR FLAGS
#define _XML_MIRROR_ELEM_FLAGS "Flags"
	#define _XML_MIRROR_ELEM_FLAG_PLANE "Plane"
	#define _XML_MIRROR_ELEM_FLAG_MASK "Mask"
	#define _XML_MIRROR_ELEM_FLAG_INPUT "Input"

//Mirror Plane
#define _XML_MIRROR_ELEM_PLANE "Plane"
	#define _XML_MIRROR_ELEM_PLANE_A "A"
		//_XML_VALUE
	#define _XML_MIRROR_ELEM_PLANE_B "B"
		//_XML_VALUE
	#define _XML_MIRROR_ELEM_PLANE_C "C"
		//_XML_VALUE
	#define _XML_MIRROR_ELEM_PLANE_D "D"
		//_XML_VALUE

//Mirror Area
#define _XML_MIRROR_ELEM_AREA "Area"
	#define _XML_MIRROR_ELEM_AREA_MASK "AreaMask"
		//_XML_PATH
	#define _XML_MIRROR_ELEM_AREA_LIMITS "Limits"
		#define _XML_MIRROR_ELEM_AREA_MAX_WIDTH "MaxWidth"
			//_XML_VALUE
		#define _XML_MIRROR_ELEM_AREA_MIN_WIDTH "MinWidth"
			//_XML_VALUE
		#define _XML_MIRROR_ELEM_AREA_MAX_HEIGHT "MaxHeight"
			//_XML_VALUE
		#define _XML_MIRROR_ELEM_AREA_MIN_HEIGHT "MinHeight"
			//_XML_VALUE

//Mirror Input
#define _XML_MIRROR_ELEM_INPUT "Input"
	#define _XML_MIRROR_ELEM_INPUT_N "NPoints"
		//_XML_VALUE
	#define _XML_MIRROR_ELEM_INPUT_POINTS "Points"
		#define _XML_MIRROR_ELEM_INPUT_POINT "Point"
		//CV::POINT

#endif //_ARENA_XML