#include <dpsim-models/Signal/DCLinkSource.h>
#include <dpsim-models/MathUtils.h>
        
using namespace CPS;
using namespace CPS::Signal;

Signal::DCLinkSource::DCLinkSource(const String & name, CPS::Logger::Level logLevel):
    SimSignalComp(name, name, logLevel),
    mIdc_s(mAttributes->create<Real>("Idc_s", 0)),
	mVdc(mAttributes->create<Real>("V_dc", 0)),
	mP_inv(mAttributes->create<Real>("P_inv", 0)) { }

void DCLinkSource::setParameters(std::shared_ptr<DCLinkSourceParameters> parameters) {
    if (auto params = std::dynamic_pointer_cast<Signal::DCLinkSourceParameters>(parameters)){
        mParameters = parameters;
        
        SPDLOG_LOGGER_INFO(mSLog, 
			"\nDC Link Source parameter:"
			"\nKpdc: {:e}"
			"\nKidc: {:e}"
			"\nCdc: {:e}"
            "\nVdcnom: {:e}",
			mParameters->Kpdc, mParameters->Kidc, 
            mParameters->Cdc, mParameters->Vdcnom);
	} else {
		std::cout << "Type of parameters class of " << this->name() << " has to be DCLinkSourceParameters!" << std::endl;
		throw CPS::TypeException();
	}
}

void DCLinkSource::initialize(Real Pdc, Real timeStep) {
    
    mTimeStep=timeStep;

    //steady state values
    **mVdc = mParameters->Vdcnom;
    mVdc_next=**mVdc;

    **mP_inv=Pdc;
    mIdc_inv = Pdc / **mVdc;

    mX=mIdc_inv;
    mX_next=mIdc_inv;

    **mIdc_s=mIdc_inv;
    mIdc_s_next=**mIdc_s;

    SPDLOG_LOGGER_INFO(mSLog, "DC Link initial values: \n"
		"\nUdc: {:f}"
		"\nIdc_inv: {:f}"
		"\nIdc_s: {:f}",
		**mVdc, mIdc_inv, **mIdc_s);
}

void DCLinkSource::step(Real Pdc) {

    **mP_inv=Pdc;
    **mVdc=mVdc_next;
    mX=mX_next;

    **mIdc_s= (mParameters->Vdcnom-**mVdc)*mParameters->Kpdc+mX;
    mIdc_inv=Pdc / **mVdc;

    mVdc_next=**mVdc + mTimeStep/mParameters->Cdc *(**mIdc_s - mIdc_inv);

    mX_next=mX + mTimeStep*mParameters->Kidc *(mParameters->Vdcnom - **mVdc);

}