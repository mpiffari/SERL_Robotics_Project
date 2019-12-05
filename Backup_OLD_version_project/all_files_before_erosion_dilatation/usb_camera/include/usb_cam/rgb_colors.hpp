#ifndef RGB_COLORS_H_
#define RGB_COLORS_H_

#include <opencv2/opencv.hpp>

 namespace rgb_colors
 {

enum Colors {
     ALICEBLUE,
     ANTIQUEWHITE,
     AQUA,
     AQUAMARINE,
     AZURE,
     BEIGE,
     BISQUE,
     BLACK,
     BLANCHEDALMOND,
     BLUE,
     BLUEVIOLET,
     BROWN,
     BURLYWOOD,
     CADETBLUE,
     CHARTREUSE,
     CHOCOLATE,
     CORAL,
     CORNFLOWERBLUE,
     CORNSILK,
     CRIMSON,
     CYAN,
     DARKBLUE,
     DARKCYAN,
     DARKGOLDENROD,
     DARKGRAY,
     DARKGREEN,
     DARKGREY,
     DARKKHAKI,
     DARKMAGENTA,
     DARKOLIVEGREEN,
     DARKORANGE,
     DARKORCHID,
     DARKRED,
     DARKSALMON,
     DARKSEAGREEN,
     DARKSLATEBLUE,
     DARKSLATEGRAY,
     DARKSLATEGREY,
     DARKTURQUOISE,
     DARKVIOLET,
     DEEPPINK,
     DEEPSKYBLUE,
     DIMGRAY,
     DIMGREY,
     DODGERBLUE,
     FIREBRICK,
     FLORALWHITE,
     FORESTGREEN,
     FUCHSIA,
     GAINSBORO,
     GHOSTWHITE,
     GOLD,
     GOLDENROD,
     GRAY,
     GREEN,
     GREENYELLOW,
     GREY,
     HONEYDEW,
     HOTPINK,
     INDIANRED,
     INDIGO,
     IVORY,
     KHAKI,
     LAVENDER,
     LAVENDERBLUSH,
     LAWNGREEN,
     LEMONCHIFFON,
     LIGHTBLUE,
     LIGHTCORAL,
     LIGHTCYAN,
     LIGHTGOLDENRODYELLOW,
     LIGHTGRAY,
     LIGHTGREEN,
     LIGHTGREY,
     LIGHTPINK,
     LIGHTSALMON,
     LIGHTSEAGREEN,
     LIGHTSKYBLUE,
     LIGHTSLATEGRAY,
     LIGHTSLATEGREY,
     LIGHTSTEELBLUE,
     LIGHTYELLOW,
     LIME,
     LIMEGREEN,
     LINEN,
     MAGENTA,
     MAROON,
     MEDIUMAQUAMARINE,
     MEDIUMBLUE,
     MEDIUMORCHID,
     MEDIUMPURPLE,
     MEDIUMSEAGREEN,
     MEDIUMSLATEBLUE,
     MEDIUMSPRINGGREEN,
     MEDIUMTURQUOISE,
     MEDIUMVIOLETRED,
     MIDNIGHTBLUE,
     MINTCREAM,
     MISTYROSE,
     MOCCASIN,
     NAVAJOWHITE,
     NAVY,
     OLDLACE,
     OLIVE,
     OLIVEDRAB,
     ORANGE,
     ORANGERED,
     ORCHID,
     PALEGOLDENROD,
     PALEGREEN,
     PALEVIOLETRED,
     PAPAYAWHIP,
     PEACHPUFF,
     PERU,
     PINK,
     PLUM,
     POWDERBLUE,
     PURPLE,
     RED,
     ROSYBROWN,
     ROYALBLUE,
     SADDLEBROWN,
     SALMON,
     SANDYBROWN,
     SEAGREEN,
     SEASHELL,
     SIENNA,
     SILVER,
     SKYBLUE,
     SLATEBLUE,
     SLATEGRAY,
     SLATEGREY,
     SNOW,
     SPRINGGREEN,
     STEELBLUE,
     TAN,
     TEAL,
     THISTLE,
     TOMATO,
     TURQUOISE,
     VIOLET,
     WHEAT,
     WHITE,
     WHITESMOKE,
     YELLOW,
     YELLOWGREEN,
   };

   cv::Vec3d getRGBColor(const int color);
} // namespace rgb_colors

#endif
