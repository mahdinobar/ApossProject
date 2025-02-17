#include <SysDef.mh>

#include "SDK_ApossC.mc"

long main(void)
{
		sdkInfoPrintSoftware();
		sdkInfoPrintHardware();
		sdkInfoPrintAxesPos();
		sdkInfoPrintPosPID();
    return(0);
}