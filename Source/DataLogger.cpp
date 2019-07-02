/**
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <iomanip>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

#include <dpsim/DataLogger.h>
#include <cps/Logger.h>

using namespace DPsim;

DataLogger::DataLogger(Bool enabled) :
	mLogFile(),
	mEnabled(enabled),
	mDownsampling(1) {
	mLogFile.setstate(std::ios_base::badbit);
}

DataLogger::DataLogger(String name, Bool enabled, UInt downsampling) :
	mName(name),
	mEnabled(enabled),
	mDownsampling(downsampling) {
	if (!mEnabled)
		return;

	String filename = CPS::Logger::logDir() + "/" + name + ".csv";

	fs::path p = filename;

	if (p.has_parent_path() && !fs::exists(p.parent_path()))
		fs::create_directory(p.parent_path());

	mLogFile = std::ofstream(filename);
	if (!mLogFile.is_open()) {
		// TODO: replace by exception
		std::cerr << "Cannot open log file " << filename << std::endl;
		mEnabled = false;
	}
}

DataLogger::~DataLogger() {
	if (mLogFile.is_open())
		mLogFile.close();
}

void DataLogger::close() {
	mLogFile.close();
}

void DataLogger::setColumnNames(std::vector<String> names) {
	if (mLogFile.tellp() == std::ofstream::pos_type(0)) {
		mLogFile << std::right << std::setw(14) << "time";
		for (auto name : names) {
			mLogFile << ", " << std::right << std::setw(13) << name;
		}
		mLogFile << '\n';
	}
}

void DataLogger::logDataLine(Real time, Real data) {
	if (!mEnabled)
		return;

	mLogFile << std::scientific << std::right << std::setw(14) << time;
	mLogFile << ", " << std::right << std::setw(13) << data;
	mLogFile << '\n';
}

void DataLogger::logDataLine(Real time, const Matrix& data) {
	if (!mEnabled)
		return;

	mLogFile << std::scientific << std::right << std::setw(14) << time;
	for (Int i = 0; i < data.rows(); i++) {
		mLogFile << ", " << std::right << std::setw(13) << data(i, 0);
	}
	mLogFile << '\n';
}

void DataLogger::logDataLine(Real time, const MatrixComp& data) {
	if (!mEnabled)
		return;
	mLogFile << std::scientific << std::right << std::setw(14) << time;
	for (Int i = 0; i < data.rows(); i++) {
		mLogFile << ", " << std::right << std::setw(13) << data(i, 0);
	}
	mLogFile << '\n';
}

void DataLogger::logPhasorNodeValues(Real time, const Matrix& data) {
	if (mLogFile.tellp() == std::ofstream::pos_type(0)) {
		std::vector<String> names;
		for (Int i = 0; i < data.rows(); i++) {
			std::stringstream name;
			if (i < data.rows() / 2)
				name << "node" << std::setfill('0') << std::setw(5) << i << ".re";
			else
				name << "node" << std::setfill('0') << std::setw(5) << (i - data.rows() / 2) << ".im";
			names.push_back(name.str());
		}
		setColumnNames(names);
	}
	logDataLine(time, data);
}

void DataLogger::logEMTNodeValues(Real time, const Matrix& data) {
	if (mLogFile.tellp() == std::ofstream::pos_type(0)) {
		std::vector<String> names;
		for (Int i = 0; i < data.rows(); i++) {
			std::stringstream name;
			name << "node" << std::setfill('0') << std::setw(5) << i;
			names.push_back(name.str());
		}
		setColumnNames(names);
	}
	logDataLine(time, data);
}

void DataLogger::log(Real time, Int timeStepCount) {
	if (!mEnabled || !(timeStepCount % mDownsampling == 0))
		return;

	if (mLogFile.tellp() == std::ofstream::pos_type(0)) {
		mLogFile << std::right << std::setw(14) << "time";
		for (auto it : mAttributes)
			mLogFile << ", " << std::right << std::setw(13) << it.first;
		mLogFile << '\n';
	}

	mLogFile << std::scientific << std::right << std::setw(14) << time;
	for (auto it : mAttributes)
		mLogFile << ", " << std::right << std::setw(13) << it.second->toString();
	mLogFile << '\n';
}

void DataLogger::Step::execute(Real time, Int timeStepCount) {
	mLogger.log(time, timeStepCount);
}

CPS::Task::Ptr DataLogger::getTask() {
	return std::make_shared<DataLogger::Step>(*this);
}

void DataLogger::addAttribute(const String &name, CPS::Attribute<Int>::Ptr attr) {
	mAttributes[name] = attr;
}

void DataLogger::addAttribute(const String &name, CPS::Attribute<Real>::Ptr attr) {
	mAttributes[name] = attr;
}

void DataLogger::addAttribute(const String &name, CPS::Attribute<Complex>::Ptr attr) {
	auto attrComp = std::static_pointer_cast<CPS::ComplexAttribute>(attr);

	mAttributes[name + ".re"] = attrComp->real();
	mAttributes[name + ".im"] = attrComp->imag();
}

void DataLogger::addAttribute(const String &name, CPS::MatrixRealAttribute::Ptr attr) {
	const Matrix &m = attr->get();
	auto attrMat = std::static_pointer_cast<CPS::MatrixRealAttribute>(attr);

	if (m.rows() == 1 && m.cols() == 1) {
		addAttribute(name, attrMat->coeff(0, 0));
	}
	else if (m.cols() == 1) {
		for (UInt k = 0; k < m.rows(); k++) {
			addAttribute(name + "(" + std::to_string(k) + ")", attrMat->coeff(k, 0));
		}
	}
	else {
		for (UInt k = 0; k < m.rows(); k++) {
			for (UInt l = 0; l < m.cols(); l++) {
				addAttribute(name + "(" + std::to_string(k) + ", " + std::to_string(l) + ")", attrMat->coeff(k, l));
			}
		}
	}
}

void DataLogger::addAttribute(const String &name, CPS::MatrixCompAttribute::Ptr attr, UInt rowsMax, UInt colsMax) {
	const MatrixVar<Complex> &m = attr->get();
	auto attrMat = std::static_pointer_cast<CPS::MatrixCompAttribute>(attr);
	if (rowsMax == 0 || rowsMax > m.rows()) rowsMax = m.rows();
	if (colsMax == 0 || colsMax > m.cols()) colsMax = m.cols();

	if (m.rows() == 1 && m.cols() == 1) {
		//addAttribute(name, attrMat->coeff(0, 0));
		mAttributes[name + ".re"] = attrMat->coeffReal(0,0);
		mAttributes[name + ".im"] = attrMat->coeffImag(0,0);
	}
	else if (m.cols() == 1) {
		for (UInt k = 0; k < rowsMax; k++) {
			//addAttribute(name + "(" + std::to_string(k) + ")", attrMat->coeff(k, 0));
			mAttributes[name + "_" + std::to_string(k) + ".re"] = attrMat->coeffReal(k,0);
			mAttributes[name + "_" + std::to_string(k) + ".im"] = attrMat->coeffImag(k,0);
		}
	}
	else {
		for (UInt k = 0; k < rowsMax; k++) {
			for (UInt l = 0; l < colsMax; l++) {
				mAttributes[name + "_" + std::to_string(k) + "_" + std::to_string(l)
					+ ".re"] = attrMat->coeffReal(k,l);
				mAttributes[name + "_" + std::to_string(k) + "_" + std::to_string(l)
					+ ".im"] = attrMat->coeffImag(k,l);
			}
		}
	}
}

void DataLogger::addAttribute(const String &name, CPS::AttributeBase::Ptr attr) {
	auto intAttr = std::dynamic_pointer_cast<CPS::Attribute<Int>>(attr);
	if (intAttr) {
		addAttribute(name, intAttr);
		return;
	}

	auto realAttr = std::dynamic_pointer_cast<CPS::Attribute<Real>>(attr);
	if (realAttr) {
		addAttribute(name, realAttr);
		return;
	}

	auto compAttr = std::dynamic_pointer_cast<CPS::Attribute<Complex>>(attr);
	if (compAttr) {
		addAttribute(name, compAttr);
		return;
	}

	//auto realMatAttr = std::dynamic_pointer_cast<CPS::Attribute<MatrixVar<Real>>>(attr);
	//if (realMatAttr) {
	//	addAttribute(name, realMatAttr);
	//	return;
	//}

	auto realMatAttr = std::dynamic_pointer_cast<CPS::Attribute<MatrixVar<Real>>>(attr);
	if (realMatAttr) {
		auto realMatAttrCast = std::static_pointer_cast<CPS::MatrixRealAttribute>(attr);
		addAttribute(name, realMatAttrCast);
		return;
	}

	//auto compMatAttr = std::dynamic_pointer_cast<CPS::Attribute<MatrixVar<Complex>>>(attr);
	//if (compMatAttr) {
	//	addAttribute(name, compMatAttr);
	//	return;
	//}

	auto compMatAttr = std::dynamic_pointer_cast<CPS::Attribute<MatrixVar<Complex>>>(attr);
	if (compMatAttr) {
		auto compMatAttrCast = std::static_pointer_cast<CPS::MatrixCompAttribute>(attr);
		addAttribute(name, compMatAttrCast);
		return;
	}

	throw CPS::InvalidAttributeException();
}

