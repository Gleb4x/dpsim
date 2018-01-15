/** Reference Circuits
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 *
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include "Simulation.h"

using namespace DPsim;
using namespace DPsim::Components::DP;

int main(int argc, char* argv[]) {

	// Define simulation scenario
	Real timeStep = 0.00005;
	Real omega = 2.0*M_PI*50.0;
	Real finalTime = 0.1;
	String simName = "IdealVS_Trafo_" + std::to_string(timeStep);

	Components::Base::List comps = {
		VoltageSource::make("v_1", GND, 0, std::polar<double>(10., 0.), Logger::Level::DEBUG),
		Inductor::make("l_1", 0, 1, 0.001, Logger::Level::DEBUG),
		Resistor::make("r_2", 1, GND, 1, Logger::Level::DEBUG),
		TransformerIdeal::make("trafo_1", 1, 2, 10, 0, Logger::Level::DEBUG),
		Resistor::make("r_1", 1, GND, 1, Logger::Level::DEBUG)
	};

	// Set up simulation and start main simulation loop
	Simulation newSim(simName, comps, omega, timeStep, finalTime);
	newSim.run();

	return 0;
}