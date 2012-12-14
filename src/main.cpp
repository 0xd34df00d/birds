#include "image.h"

int main (int argc, char **argv)
{
	Image img ("data/п_1.txt");
	img.PrintPseudoHull ();
	img.PrintSkeleton ();
}
