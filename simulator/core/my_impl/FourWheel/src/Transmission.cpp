#include "Transmission.h"
using namespace std;
void Transmission::transinit()
{
	clutch = true;
	k=2;
};
void Transmission::autotrans()
{
	//cout << "enginespeed   " << n << endl;
	if ((n < 1500 && k != 2) || (n > 3000 && k != 7))
	{
		if (n<800)
		{
			k=2;
			n=800;
		}
		else if (n > 3000 && k != 7)
		{
			k++;
			n = i[k] / i[k - 1] * n;
		}
		else if (n < 1500  && k != 2)
		{
			k--;
			n = i[k] / i[k + 1] * n;
		}
		else
		{
			//k = 1;
		}

	
	}
	i_ = i[k];
};

