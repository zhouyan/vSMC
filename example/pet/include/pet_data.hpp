//============================================================================
// vSMC/vSMCExample/pet/include/pet_data.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

double Decay = 0;
std::size_t DataSetNum = 0;
std::size_t DataStart = 0;
std::size_t DataStop = 0;
std::size_t DataNum = 0;
std::size_t ModelNum = 0;
std::size_t ConvNum = 0;
double ConvMul = 0;
std::string TimeFile = 0;
std::string ConvFile = 0;
std::string PriorFile = 0;
std::string SDFile = 0;
bool UseStudentT = false;

std::ifstream config_file;

config_file.open(DataFile.c_str());
config_file >> Decay >> DataSetNum >> DataStart >> DataStop
    >> ModelNum >> DataNum >> ConvNum >> ConvMul
    >> DataFile >> TimeFile >> ConvFile >> PriorFile >> SDFile >> UseStudentT;
config_file.close();
config_file.clear();

PETModel ModelType = Normal;
if (UseStudentT)
    ModelType = StudentT;

std::ifstream data_file;

std::vector<double> Data(DataNum * DataSetNum);
data_file.open(DataFile.c_str());
for (std::size_t r = 0; r != DataSetNum; ++r)
    for (std::size_t c = 0; c != DataNum; ++c)
        data_file >> Data[c + r * DataNum];
data_file.close();
data_file.clear();

std::vector<double> Time(DataNum);
data_file.open(TimeFile.c_str());
for (std::size_t i = 0; i != DataNum; ++i)
    data_file >> Time[i];
data_file.close();
data_file.clear();

std::vector<double> Conv(DataNum * ConvNum);
data_file.open(ConvFile.c_str());
for (std::size_t r = 0; r != ConvNum; ++r)
    for (std::size_t c = 0; c != DataNum; ++c)
        data_file >> Conv[c + r * DataNum];
data_file.close();
data_file.clear();

std::vector<double> Prior(ModelNum * 8 + 4);
std::size_t prior_offset = 0;
data_file.open(PriorFile.c_str());
for (std::size_t i = 0; i != ModelNum; ++i)
    data_file >> Prior[prior_offset++]; pet_ignore(data_file); // phi_lb0
for (std::size_t i = 0; i != ModelNum; ++i)
    data_file >> Prior[prior_offset++]; pet_ignore(data_file); // phi_ub0
for (std::size_t i = 0; i != ModelNum; ++i)
    data_file >> Prior[prior_offset++]; pet_ignore(data_file); // theta_lb0
for (std::size_t i = 0; i != ModelNum; ++i)
    data_file >> Prior[prior_offset++]; pet_ignore(data_file); // theta_ub0
data_file >> Prior[prior_offset++]; pet_ignore(data_file); // lambda_a0
data_file >> Prior[prior_offset++]; pet_ignore(data_file); // lambda_b0
data_file >> Prior[prior_offset++]; pet_ignore(data_file); // nu_a0
data_file >> Prior[prior_offset++]; pet_ignore(data_file); // nu_b0
data_file.close();
data_file.clear();

std::vector<double> SD(ModelNum * 2 + 2);
std::size_t sd_offset = 0;
data_file.open(SDFile.c_str());
for (std::size_t i = 0; i != ModelNum; ++i)
    data_file >> SD[sd_offset++]; pet_ignore(data_file); // phi_sd
for (std::size_t i = 0; i != ModelNum; ++i)
    data_file >> SD[sd_offset++]; pet_ignore(data_file); // theta_sd
data_file >> SD[sd_offset++]; pet_ignore(data_file); // lamda_sd;
data_file >> SD[sd_offset++]; pet_ignore(data_file); // nu_sd;
data_file.close();
data_file.clear();

data_info  info_d = {DataNum, &Data[0]};
time_info  info_t = {DataNum, &Time[0]};
conv_info  info_c = {ConvNum, ConvMul, &Conv[0]};
prior_info info_p = {ModelNum, &Prior[0]};
sd_info    info_s = {ModelNum, &SD[0]};
model_info info_m = {Decay, ModelType};
pet_info info = {
    &info_d, true,
    &info_t, true,
    &info_c, true,
    &info_p, true,
    &info_s, true,
    &info_m, true
};

if (Interactive) {
    std::cout << "Data reading is done." << std::endl;
    std::string answer;
    std::string start("start");
    while (answer != start) {
        std::cout << "Enter [start] to start computation" << std::endl;
        std::cin >> answer;
    }
}
