#include "battleship.h"
#include "boxpushing.h"
#include "mcts.h"
#include "network.h"
#include "pocman.h"
#include "rocksample.h"
#include "tag.h"
#include "experiment.h"
#include "kitchen.h"
#include <boost/program_options.hpp>

using namespace std;
using namespace boost::program_options;

void UnitTests(MCTS::PARAMS& searchParams)
{
    cout << "Testing UTILS" << endl;
    UTILS::UnitTest();
    cout << "Testing COORD" << endl;
    COORD::UnitTest();
    cout << "Testing MCTS" << endl;
    MCTS::UnitTest(0);
    if (searchParams.MultiAgent)
	MCTS::UnitTest(2);
}

void disableBufferedIO(void)
{
    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    setbuf(stderr, NULL);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
}

int main(int argc, char* argv[])
{
    MCTS::PARAMS searchParams;
    EXPERIMENT::PARAMS expParams;
    SIMULATOR::KNOWLEDGE knowledge;
    string problem, outputfile, policy;
    int size, number, treeknowledge = 1, rolloutknowledge = 1, smarttreecount = 10;
    double smarttreevalue = 1.0;

    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("test", "run unit tests")
        ("problem", value<string>(&problem), "problem to run")
        ("outputfile", value<string>(&outputfile)->default_value("output.txt"), "summary output file")
        ("policy", value<string>(&policy), "policy file (explicit POMDPs only)")
        ("size", value<int>(&size), "size of problem (problem specific)")
        ("number", value<int>(&number), "number of elements in problem (problem specific)")
        ("timeout", value<double>(&expParams.TimeOut), "timeout (seconds)")
        ("mindoubles", value<int>(&expParams.MinDoubles), "minimum power of two simulations")
        ("maxdoubles", value<int>(&expParams.MaxDoubles), "maximum power of two simulations")
        ("runs", value<int>(&expParams.NumRuns), "number of runs")
        ("accuracy", value<double>(&expParams.Accuracy), "accuracy level used to determine horizon")
        ("horizon", value<int>(&expParams.UndiscountedHorizon), "horizon to use when not discounting")
        ("num steps", value<int>(&expParams.NumSteps), "number of steps to run when using average reward")
        ("verbose", value<int>(&searchParams.Verbose), "verbosity level")
        ("autoexploration", value<bool>(&expParams.AutoExploration), "Automatically assign UCB exploration constant")
        ("exploration", value<double>(&searchParams.ExplorationConstant), "Manual value for UCB exploration constant")
        ("usetransforms", value<bool>(&searchParams.UseTransforms), "Use transforms")
        ("transformdoubles", value<int>(&expParams.TransformDoubles), "Relative power of two for transforms compared to simulations")
        ("transformattempts", value<int>(&expParams.TransformAttempts), "Number of attempts for each transform")
        ("userave", value<bool>(&searchParams.UseRave), "RAVE")
        ("ravediscount", value<double>(&searchParams.RaveDiscount), "RAVE discount factor")
        ("raveconstant", value<double>(&searchParams.RaveConstant), "RAVE bias constant")
        ("treeknowledge", value<int>(&knowledge.TreeLevel), "Knowledge level in tree (0=Pure, 1=Legal, 2=Smart)")
        ("rolloutknowledge", value<int>(&knowledge.RolloutLevel), "Knowledge level in rollouts (0=Pure, 1=Legal, 2=Smart)")
        ("smarttreecount", value<int>(&knowledge.SmartTreeCount), "Prior count for preferred actions during smart tree search")
        ("smarttreevalue", value<double>(&knowledge.SmartTreeValue), "Prior value for preferred actions during smart tree search")
        ("disabletree", value<bool>(&searchParams.DisableTree), "Use 1-ply rollout action selection")
	("multiagent", value<bool>(&searchParams.MultiAgent), "Multi-agent problem")
        ;

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help"))
    {
        cout << desc << "\n";
        return 1;
    }

    if (vm.count("problem") == 0)
    {
        cout << "No problem specified" << endl;
        return 1;
    }

    if (vm.count("test"))
    {
        cout << "Running unit tests" << endl;
        UnitTests(searchParams);
        return 0;
    }

    SIMULATOR* real = 0;
    SIMULATOR* simulator = 0;

    if (problem == "battleship")
    {
        real = new BATTLESHIP(size, size, number);
        simulator = new BATTLESHIP(size, size, number);
    }
    else if (problem == "pocman")
    {
        real = new FULL_POCMAN;
        simulator = new FULL_POCMAN;
    }
    else if (problem == "network")
    {
        real = new NETWORK(size, number);
        simulator = new NETWORK(size, number);
    }
    else if (problem == "rocksample")
    {
        real = new ROCKSAMPLE(size, number);
        simulator = new ROCKSAMPLE(size, number);
    }
    else if (problem == "tag")
    {
        real = new TAG(number);
        simulator = new TAG(number);
    }
    else if (problem == "boxpushing")
    {
        real = new BOXPUSHING(size, size-1, number, number-1);
        simulator = new BOXPUSHING(size, size-1, number, number-1);
    }
    else if (problem == "kitchen")
    {
	real = new KITCHEN(size, number);
	simulator = new KITCHEN(size, number);
    }
    else 
    {
        cout << "Unknown problem" << endl;
        exit(1);
    }


    simulator->SetKnowledge(knowledge);
    EXPERIMENT experiment(*real, *simulator, outputfile, expParams, searchParams);
    experiment.DiscountedReturn();

    delete real;
    delete simulator;
    return 0;
}
