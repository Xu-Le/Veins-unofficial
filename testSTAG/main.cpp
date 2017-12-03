//
// Copyright (C) 2017 Xu Le <xmutongxinXuLe@163.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "STAG.h"

extern int log_level;

extern int nodeNum;
extern int linkNum;
extern int downloaderNum;
extern int slotNum;

extern std::map<int, std::string> storageTable;

static void printHelp()
{
	printf("Usage:\n    ./STAG cases/case_x.txt results/result_x.txt\n");
	printf("Example:\n    ./STAG cases/case_1.txt results/result_1.txt\n");
}

int main(int argc, char *argv[])
{
	if (argc != 3)
	{
		printHelp();
		exit(EXIT_FAILURE);
	}

	printf("Current log level: %s\n\n", getLogLevel());

	std::list<LinkTuple> linkTuples;
	parseInput(argv[1], linkTuples);
	// printInput(linkTuples);

	STAG graph;
	graph.construct(linkTuples);
	graph.undirectify(linkTuples);
	// graph.display();

	linkTuples.clear();

	{
		Timer algorithmTimer("Algorithm Time Cost: ");
		int result = graph.maximumFlow();
		info_log("Final total received data amount is %d.\n", result);
		// graph.test();
	}

#if VERIFY_SCHEME
	int totalReceivedAmount = 0;
	bool schemeOK = verifyScheme(graph, totalReceivedAmount);
	if (schemeOK == true)
		uncond_log("Congratulations! Total total received data amount is %d.\n", totalReceivedAmount);
	else
		error_log("Shit! The scheme failed to pass verification!\n");

	outputToFile(argv[2], graph.fluxPathList);
#endif

	graph.destruct();

	return 0;
}
