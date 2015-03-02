#include <stdio.h>
#include <stdlib.h>
#include <wand/MagickWand.h>

/* Lesson learned:
 The purpose of this file is to test if MagickQueryFontMetrics()
 returns non-NULL value. It does. So I think ths sbug1 has
 weird setup of ImageMagick or it has not enough compilation option
 (Refer to makefileImageMagickX)
*/

int main (int argc, char **argv){
	MagickWandGenesis();	// must call in every use.

	MagickWand *mw = NewMagickWand();	// magic wand is used for everything.
	DrawingWand *dw = NewDrawingWand(); 	// drawing wand contains font information, such as font size, font type.
	const char *text = "hello";		// the text we want to display as an image.
	MagickReadImage(mw, "xc:");		// Assosciate a wand with an image. in this case "xc:" means throw-away image
	DrawSetFontSize(dw, 40);
	DrawSetFont(dw, "Arial");

	double * fontmetrics;
	fontmetrics = MagickQueryFontMetrics(mw, dw, text);	// The function returns the info about the text's imagified object info.

	mw = DestroyMagickWand(mw);
	MagickWandTerminus();

	return 0;
}
