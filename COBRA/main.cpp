#include "Simulation.h"

int main(int argc, char** argv)
{
    Simulation simu1(argv[1], argv[2]);
    simu1.run_TOTP();
//    simu1.SaveThroughput((string)argv[2] + "_tp_throughput");
//    simu1.SaveTask((string)argv[2] + "_tp_out", argv[2]);
    simu1.SavePath((string)argv[2] + "_tp_path");

	Simulation simu2(argv[1], argv[2]);
	simu2.run_TPTR();
//    simu2.SaveThroughput((string)argv[2] + "_tptr_throughput");
//    simu2.SaveTask((string)argv[2] + "_tptr_out", argv[2]);
    simu2.SavePath((string)argv[2] + "_tptr_path");

//    simu1.ShowTask();
	simu2.ShowTask();
    return 0;
}

/*#include <iostream>
#include <string>
#include "Simulation.h"

void batch_run(const std::string& inputFilePath, const std::string& outputFilePath);

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <inputFilePath> <outputFilePath>" << std::endl;
        return 1;
    }

    // Call batch_run with input and output file paths
    batch_run(argv[1], argv[2]);

    // Existing single simulation runs
    Simulation simu1(argv[1], argv[2]);
    simu1.run_TOTP();
    simu1.SavePath((std::string)argv[2] + "_tp_path");

    Simulation simu2(argv[1], argv[2]);
    simu2.run_TPTR();
    simu2.SavePath((std::string)argv[2] + "_tptr_path");
    simu2.ShowTask();

    return 0;
}*/


