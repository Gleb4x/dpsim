#include <DPsim.h>
#include <dpsim-models/Factory.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

// Grid parameters
const Examples::Grids::SMIB::ScenarioConfig3 GridParams;

// Generator parameters
const Examples::Components::SynchronousGeneratorKundur::MachineParameters syngenKundur;

// PSS
const Examples::Components::PowerSystemStabilizer::PSS1APSAT pssAndersonFarmer;

// Excitation system
const auto excitationEremia = Examples::Components::Exciter::getExciterParametersEremia();

// Steam Turbine
const Examples::Components::TurbineGovernor::SteamTurbine dSteamTurbine;

// Steam Turbine Governor
Examples::Components::TurbineGovernor::SteamTurbineGovernor dSteamGovernor;

int main(int argc, char* argv[]) {

	// initiaize factories
	ExciterFactory::registerExciters();
	SynchronGeneratorFactory::EMT::Ph3::registerSynchronGenerators();

	// Simultion parameters
	Real switchClosed = GridParams.SwitchClosed;
	Real switchOpen = GridParams.SwitchOpen;
	Real startTimeFault = 1.0;
	Real endTimeFault   = 4;
	Real finalTime = 20;
	Real timeStep = 1e-3;
	Real H = syngenKundur.H;
<<<<<<< HEAD
	bool withPSS = false;
	bool withExciter = false;
	bool withTurbineGovernor = false;
=======
	bool withExciter = true;
	bool withTurbineGovernor = true;
>>>>>>> 8e9cbf324 (HiWi added new Hydro and Steam Turbines and Governor models)
	std::string SGModel = "4";
	std::string stepSize_str = "";
	std::string inertia_str = "";

	// Command line args processing
	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		if (args.options.find("SGModel") != args.options.end())
			SGModel = args.getOptionString("SGModel");
		if (args.options.find("WITHPSS") != args.options.end())
			withPSS = args.getOptionBool("WITHPSS");
		if (args.options.find("WITHEXCITER") != args.options.end())
			withExciter = args.getOptionBool("WITHEXCITER");		
		if (args.options.find("WithTurbineGovernor") != args.options.end())
			withTurbineGovernor = args.getOptionBool("WithTurbineGovernor");
		if (args.options.find("StepSize") != args.options.end()) {
			timeStep = args.getOptionReal("StepSize");
			stepSize_str = "_StepSize_" + std::to_string(timeStep);
		}
		if (args.options.find("Inertia") != args.options.end())  {
			H = args.getOptionReal("Inertia");
			inertia_str = "_Inertia_" + std::to_string(H);
		}
	}

	Real logDownSampling;
	if (timeStep<100e-6)
		logDownSampling = floor(100e-6 / timeStep);
	else
		logDownSampling = 1.0;
	Logger::Level logLevel = Logger::Level::debug;
	std::string simName ="EMT_SynGen" + SGModel + "Order_VBR_Load_Fault" + stepSize_str + inertia_str;


	// ----- Dynamic simulation ------
	String simNameEMT = simName;
	Logger::setLogDir("logs/"+simNameEMT);

	// Nodes
	std::vector<Complex> initialVoltage_n1{ GridParams.initTerminalVolt,
											GridParams.initTerminalVolt * SHIFT_TO_PHASE_B,
											GridParams.initTerminalVolt * SHIFT_TO_PHASE_C
										  };
	auto n1EMT = SimNode<Real>::make("n1EMT", PhaseType::ABC, initialVoltage_n1);

	// Components
	// Synchronous generator
	auto genEMT = Factory<EMT::Ph3::ReducedOrderSynchronGeneratorVBR>::get().create(SGModel, "SynGen", logLevel);
	genEMT->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll,
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
			syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s, syngenKundur.Tq0_s);
    genEMT->setInitialValues(GridParams.initComplexElectricalPower, GridParams.mechPower,
							 GridParams.initTerminalVolt);
	genEMT->setModelAsNortonSource(true);
	
	// Exciter
	std::shared_ptr<Base::Exciter> exciterEMT = nullptr;
	if (withExciter) {
		exciterEMT = Factory<Base::Exciter>::get().create("DC1Simp", "Exciter", logLevel);
		exciterEMT->setParameters(excitationEremia);
		genEMT->addExciter(exciterEMT);
	}

<<<<<<< HEAD
	// Power system stabilizer
	std::shared_ptr<Signal::PSS1A> pssEMT = nullptr;
	if (withPSS) {
		pssEMT = Signal::PSS1A::make("PSS", logLevel);
		pssEMT->setParameters(pssAndersonFarmer.Kp, pssAndersonFarmer.Kv, pssAndersonFarmer.Kw, 
			pssAndersonFarmer.T1, pssAndersonFarmer.T2, pssAndersonFarmer.T3, pssAndersonFarmer.T4, 
			pssAndersonFarmer.Vs_max, pssAndersonFarmer.Vs_min, pssAndersonFarmer.Tw);
		genEMT->addPSS(pssEMT);
	}

	// Turbine Governor
	std::shared_ptr<Signal::TurbineGovernorType1> turbineGovernorEMT = nullptr;
=======
	// Steam Turbine
	std::shared_ptr<Signal::SteamTurbine> steamTurbine = nullptr;
>>>>>>> 8e9cbf324 (HiWi added new Hydro and Steam Turbines and Governor models)
	if (withTurbineGovernor) {
		steamTurbine = Signal::SteamTurbine::make("SynGen_SteamTurbine", logLevel);
		steamTurbine->setParameters(dSteamTurbine.Fhp, dSteamTurbine.Fip,dSteamTurbine.Flp,
									dSteamTurbine.Tch, dSteamTurbine.Tco, dSteamTurbine.Trh);
		genEMT->addSteamTurbine(steamTurbine);
	}

	// Steam Turbine Governor
	std::shared_ptr<Signal::SteamTurbineGovernor> steamTurbineGovernor = nullptr;
	if (withTurbineGovernor) {
		steamTurbineGovernor = Signal::SteamTurbineGovernor::make("SynGen_SteamTurbineGovernor", logLevel);
		steamTurbineGovernor->setParameters(dSteamGovernor.OmRef, dSteamGovernor.R, dSteamGovernor.T2, dSteamGovernor.T3,  
								dSteamGovernor.delPmax, dSteamGovernor.delPmin, dSteamGovernor.Pmax, dSteamGovernor.Pmin);
		genEMT->addSteamTurbineGovernor(steamTurbineGovernor);
	}

	// Load
	auto load = CPS::EMT::Ph3::RXLoad::make("Load", logLevel);
	load->setParameters(Math::singlePhaseParameterToThreePhase(GridParams.initActivePower/3),
						Math::singlePhaseParameterToThreePhase(GridParams.initReactivePower/3),
						GridParams.VnomMV);

	//Breaker
	auto fault = CPS::EMT::Ph3::Switch::make("Br_fault", logLevel);
	fault->setParameters(Math::singlePhaseParameterToThreePhase(switchOpen),
						 Math::singlePhaseParameterToThreePhase(switchClosed));
	fault->openSwitch();

	// Topology
	genEMT->connect({ n1EMT });
	load->connect({ n1EMT });
	fault->connect({EMT::SimNode::GND, n1EMT});

	auto systemEMT = SystemTopology(GridParams.nomFreq,
			SystemNodeList{n1EMT},
			SystemComponentList{genEMT, load, fault});

	// Logging
	auto loggerEMT = DataLogger::make(simNameEMT, true, logDownSampling);
	loggerEMT->logAttribute("v_gen", 	genEMT->attribute("v_intf"));
	loggerEMT->logAttribute("i_gen", 	genEMT->attribute("i_intf"));
    loggerEMT->logAttribute("Te",		genEMT->attribute("Te"));
    loggerEMT->logAttribute("delta", 	genEMT->attribute("delta"));
    loggerEMT->logAttribute("w_r", 		genEMT->attribute("w_r"));
	loggerEMT->logAttribute("Vdq0", 	genEMT->attribute("Vdq0"));
	loggerEMT->logAttribute("Idq0", 	genEMT->attribute("Idq0"));
	loggerEMT->logAttribute("Ef",   	genEMT->attribute("Ef"));
	loggerEMT->logAttribute("Tm", 		genEMT->attribute("Tm"));

	Simulation simEMT(simNameEMT, logLevel);
	simEMT.doInitFromNodesAndTerminals(true);
	simEMT.setSystem(systemEMT);
	simEMT.setTimeStep(timeStep);
	simEMT.setFinalTime(finalTime);
	simEMT.setDomain(Domain::EMT);
	simEMT.addLogger(loggerEMT);
	simEMT.doSystemMatrixRecomputation(true);

	// Events
	auto sw1 = SwitchEvent3Ph::make(startTimeFault, fault, true);
	simEMT.addEvent(sw1);
	auto sw2 = SwitchEvent3Ph::make(endTimeFault, fault, false);
	simEMT.addEvent(sw2);

	simEMT.run();
	simEMT.logStepTimes(simNameEMT + "_step_times");
}
