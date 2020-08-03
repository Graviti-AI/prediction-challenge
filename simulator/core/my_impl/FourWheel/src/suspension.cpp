#include <iostream>
#include "suspension.h"
#include "singletire.h"


void susp::suspini(double suspK_, double suspC_)
{
	suspC = suspC_;
	suspK = suspK_;
	suspension.Fzsusp = 0;
	suspension.Zsusp = 0;
	suspension.Z_dot = 0;

}

void susp::suspstate()
{
	suspensionpre = suspension;
	suspension.Zsusp =  - z;
	suspension.Z_dot =  - z_dot;

	//std::cout << "nvkjfdsh" << std::endl;
    //std::cout << suspension.Zsusp << std::endl;
    //std::cout << suspK << std::endl;
    //std::cout << suspension.Z_dot << std::endl;
    //std::cout << suspC << std::endl;

	if (suspension.Zsusp<0.1&&suspension.Zsusp>-0.1)
    {
		suspension.Fzsusp = suspension.Zsusp * suspK + suspension.Z_dot * suspC;
	}
	else if (suspension.Zsusp>=0.1)
	{
		suspension.Fzsusp = 0.1*suspK+20*(suspension.Zsusp-0.1) * suspK + suspension.Z_dot * suspC;
	}
	else
	{
		suspension.Fzsusp = -0.1*suspK+20*(suspension.Zsusp+0.1) * suspK + suspension.Z_dot * suspC;
	}
}