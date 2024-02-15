#pragma once

#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Logger.h>

namespace CPS {
namespace Signal {

    class DCLinkSourceParameters :
		public SharedFactory<DCLinkSourceParameters> {
		
		public:
			/// Parameters of (Kpdc+Kidc/s) controller for the currect source
			/// Proportional gane
            Real Kpdc;
            /// Integral gain 
            Real Kidc;
            /// Capacitance of the DC Link in pu firts, and then converted to normal values
            Real Cdc;
			//Nominal DC Voltage of a DC Link capactor (2 * V_nom_phph recomended)
			Real Vdcnom;
	};

    /// DC Link 
    /// A controllable current source is connected to a DC capacitance. The model is based on the law of conservation of energy.
	/// P_dc=P_ac. To calculate the current that flows into inverter a voltage capacitance from the previous step is assumed.
	class DCLinkSource:
		public SimSignalComp,
		public SharedFactory<DCLinkSource> {

	protected:	
        /// DC Link Source Parameters
		std::shared_ptr<DCLinkSourceParameters> mParameters;

        /// DC Link 
        /// Power of low pressure stage at time step k+1
		Real mVdc;
		Real mVdc_next;

   		Real mIdc_inv;

    	Real mIdc_s;
		Real mIdc_s_next;

		Real mX;
		Real mX_next;
		
		Real mTimeStep;

    public:
        ///
        explicit DCLinkSource(const String & name) : SimSignalComp(name, name) { }

	    /// Constructor with log level
	    DCLinkSource(const String & name, CPS::Logger::Level logLevel);

	    /// Sets Parameters of the turbine
	    void setParameters(std::shared_ptr<DCLinkSourceParameters> parameters) ;

	    /// Initialises the initial state of the turbine
	    void initialize(Real Pdc, Real timeStep);

	    /// Performs a step to update all state variables and the output
	    Real step(Real Pdc);
    };

}
}