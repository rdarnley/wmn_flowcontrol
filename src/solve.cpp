#include "solve.h"

void processResult(const vector<vector<SolverOutput>>& result){

    std::ofstream myFileResults("experimentResults.csv");
    int expCt = 0;
    for (auto solns: result){

        std::cout << "For experiment " << expCt << ", results are - " << std::endl;

        for (auto soln: solns){

            std::cout << "Solver Type - " << soln.solverType << ", Ordering is " << soln.orderingType;
            std::cout << ", Time steps are " << soln.numTimeSteps << ", Runtime = " << soln.runtime;
            std::cout << ", Cost = " << soln.cost << std::endl;

            myFileResults << expCt << "," << soln.solverType << "," << soln.orderingType << "," << soln.numTimeSteps << ",";
            myFileResults << soln.runtime << "," << soln.cost << "\n";
        } // end inner for loop

        myFileResults << "\n";
        expCt++;
    } // end outer for loop

    myFileResults.close();

} // end processResult

FGSolver::FGSolver(){

} // end FGSolver Constructor

void FGSolver::init(uint32_t _numTimeSteps, uint32_t _numNodes, uint32_t _numControl, uint32_t _initBuffer, uint32_t _initRate, uint32_t _finalBuffer, uint32_t _finalRate){

    std::cout << "Initializing FGSolver" << std::endl;

    // Parse Input
    numTimeSteps = _numTimeSteps;
    numNodes = _numNodes;
    numControl = _numControl;
    initBuffer = _initBuffer;
    initRate = _initRate;
    finalBuffer = _finalBuffer;
    finalRate = _finalRate;

    // Other Variables
    numStates = 2;      // buffer, velocity
    timeStep = 0.05;    // modify later??

    qCost = 10;
    qfCost = 100;
    rCost = 1;
    uEstimate = gtsam::Vector(1); uEstimate << 0.1;

    // Create Constrained Noise Models
    priorNoise = noiseModel::Constrained::All(numStates);
    dynNoise = noiseModel::Constrained::All(numStates);

    // Set Prior and Final States
    priorState = gtsam::Vector(numStates); priorState << initBuffer, initRate;
    finalState = gtsam::Vector(numStates); finalState << finalBuffer, finalRate;

    // Initialize Full Matrices For Matrix Comparison
    A_full = MatrixXd::Zero(numStates * numNodes, numStates * numNodes);
    B_full = MatrixXd::Zero(numStates * numNodes, numControl);
    Q_full = MatrixXd::Zero(numStates * numNodes, numStates * numNodes);
    Qf_full = MatrixXd::Zero(numStates * numNodes, numStates * numNodes);
    R_full = MatrixXd::Zero(numControl, numControl);

    // Create map of actuated nodes
    driverIdx = unordered_map<int, int>();
    for(int i=0; i<numControl; i++){
        driverIdx[i*numNodes / numControl] = i;
    } // end driverIdx for loop

    // Create Initial State Vector
    X_init = MatrixXd::Zero(numNodes*2, 1);
    
    for(int i=0; i<numNodes; i++){
        X_init(2*i, 0) = priorState(0);
        X_init(2*i+1, 0) = priorState(1);
    } // end population for loop

    // Initialize Factor Graph
    graph = NonlinearFactorGraph();
    initialEstimate = Values();

    // Add Nodes To Factor Graph
    vector<Key> temp;
    X = vector<vector<Key>>(numNodes, temp);

    vector<Key> tempU;
    U = vector<vector<Key>>(numControl, tempU);

    std::cout << "Pushing State Nodes To FG" << std::endl;

    for(int j=0; j<numNodes; j++) {
        for(int i=0; i<numTimeSteps; i++) {
            X[j].push_back(gtsam::LabeledSymbol('x', j, i));
        } // end inner
    } // end outer

    std::cout << "Pushing Control Nodes To FG" << std::endl;

    for(int j=0; j<numControl; j++) { 
        for(int i=0; i<numTimeSteps; i++) {
            U[j].push_back(gtsam::LabeledSymbol('u', j, i));
        } // end inner
    } // end outer

    std::cout << "Pushing Prior Nodes To FG" << std::endl;

    for(int i=0; i<numNodes; i++){
        graph.add(PriorFactor<gtsam::Vector>(X[i][0], priorState, priorNoise));
    } // end prior factor for loop

    std::cout << "Adding Dynamics Constraints" << std::endl;

    MatrixXd posOne = MatrixXd::Zero(1,1);
    posOne << 1;

    MatrixXd negOne = MatrixXd::Zero(1,1);
    negOne << -1;

    MatrixXd rC = MatrixXd::Zero(1,1);
    rC << rCost;

    for(int i2 = 0; i2<numNodes; i2++) {

        // Check if node active or passive
        bool applyControl = false; int controlIdx = -1;

        if(driverIdx.find(i2) != driverIdx.end()){
            applyControl = true;
            controlIdx = driverIdx[i2];
        } // end apply control

        // Fill A Matrix
        if (i2 == 0){
            MatrixXd a = MatrixXd::Zero(1,2); a << 1,-1;
            A_full.block<1,2>(0,0) = a;
        } else {
            MatrixXd a = MatrixXd::Zero(1,3); a << 1, 1, -1;
            A_full.block<1,3>(2*i2,2*i2-1) = a;
        }

        A_full.block<1,1>(2*i2+1, 2*i2+1) = posOne;

        // Fill B Matrix
        if (applyControl) {
            B_full.block<1,1>(2*i2+1, controlIdx) = posOne;

            R_full.block<1,1>(controlIdx, controlIdx) = rC;
        }

        // Fill Q Matrix
        Q_full(2*i2, 2*i2) = qCost;
        Q_full(2*i2+1, 2*i2+1) = qCost;
        Qf_full(2*i2, 2*i2) = qfCost;
        Qf_full(2*i2+1, 2*i2+1) = qfCost;

        // Iterate Through TimeSteps (For FG Method)
        for (int i1=0; i1<numTimeSteps-1; i1++) {

            if(i2 == 0){
                if(applyControl){
                    graph.emplace_shared<active_end>(X[i2][i1], U[controlIdx][i1], X[i2][i1+1], dynNoise);
                } else if(!applyControl) {
                    graph.emplace_shared<passive_end>(X[i2][i1], X[i2][i1+1], dynNoise);
                } else {
                    std::cout << "Error: Incorrect Factor 1" << std::endl;
                }
            } else {
                if(applyControl){
                    graph.emplace_shared<active>(X[i2-1][i1], X[i2][i1], U[controlIdx][i1], X[i2][i1+1], dynNoise);
                } else if(!applyControl) {
                    graph.emplace_shared<passive>(X[i2-1][i1], X[i2][i1], X[i2][i1+1], dynNoise);
                } else {
                    std::cout << "Error: Incorrect Factor 2" << std::endl;
                }
            }

        } // end inner/temporal for loop

    } // end outer/spatial for loop

    // Create Cost Nodes
    std::cout << "Add Costs To FG" << std::endl;

    qNode = gtsam::Vector(numStates); qNode << qCost, qCost;
    qfNode = gtsam::Vector(numStates); qfNode << qfCost, qfCost;

    rNode = gtsam::Vector(1); rNode << rCost;

    qMatrix = qNode.array().matrix().asDiagonal();
    qfMatrix = qfNode.array().matrix().asDiagonal();
    rMatrix = rNode.array().matrix().asDiagonal();

    auto q_node = noiseModel::Gaussian::Information(qMatrix);
    auto qf_node = noiseModel::Gaussian::Information(qfMatrix);
    auto r_node = noiseModel::Gaussian::Information(rMatrix);

    for(int i=0; i<numNodes; i++){
        for(int j=0; j<numTimeSteps; j++){
            if(j==numTimeSteps - 1){
                graph.add(PriorFactor<gtsam::Vector>(X[i][j], finalState, qf_node));
            } else {
                graph.add(PriorFactor<gtsam::Vector>(X[i][j], finalState, q_node));
            }
            initialEstimate.insert(X[i][j], finalState);
        } // end inner
    } // end outer

    for(int i=0; i<numControl; i++){
        for(int j=0; j<numTimeSteps-1; j++){
            graph.add(PriorFactor<gtsam::Vector>(U[i][j], uEstimate, r_node));
            initialEstimate.insert(U[i][j], uEstimate);
        } // end inner
    } // end outer

    return;
}

// Solve Factor Graph With Different Variable Orderings
SolverOutput FGSolver::FGSolve(int orderingType) {
    std::cout << "Entered FG Solve" << std::endl;

    // Create Some Sort of Order
    if (orderingType == 0) {

        std::cout << "Using COLAMD Ordering" << std::endl;
        order = graph.orderingCOLAMD();

    } else if (orderingType == 1){

        std::cout << "Outside: Space, Inside: Time" << std::endl;

        // Iterate over nodes
        for(int i2=0; i2<numNodes; i2++) {

            bool applyControl = false; int controlIdx = -1;

            // Check if node is active
            if(driverIdx.find(i2) != driverIdx.end()){
                applyControl = true;
                controlIdx = driverIdx[i2];
            }

            if(applyControl){

                for(int i1=0; i1<numTimeSteps; i1++){

                    // Add State
                    order.push_back(X[i2][i1]);

                    if(i1 < numTimeSteps - 1){
                        order.push_back(U[controlIdx][i1]);
                    }
                }

            } else {

                for(int i1=0; i1<numTimeSteps; i1++){
                    // Add State
                    order.push_back(X[i2][i1]);
                }
            }
        }

    } else if (orderingType == 2){

        std::cout << "Outside: Time, Inside: Space" << std::endl;

        // Iterate over time
        for(int i2=0; i2<numTimeSteps; i2++){

            // Iterate over nodes
            for(int i1=0; i1<numNodes; i1++){

                bool applyControl = false; int controlIdx = -1;

                // Check if node is active
                if(driverIdx.find(i1) != driverIdx.end()){
                    applyControl = true;
                    controlIdx = driverIdx[i1];
                }

                order.push_back(X[i1][i2]);

                if(applyControl && i2<numTimeSteps-1){
                    order.push_back(U[controlIdx][i2]);
                }

            }

        }

    }

    // Create Instance of Optimizer
    GaussNewtonOptimizer optimizer(graph, initialEstimate, order);

    // Start Timer
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    std::cout << "Before optimize call" << std::endl;
    // Optimize Factor Graph
    Values resultLQR = optimizer.optimize();

    std::cout << "After optimize call" << std::endl;

    // End Timer
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Factor Graph Took " << duration.count() << " seconds\n" << std::endl;

    SolverOutput fgSoln(numNodes, numTimeSteps, numControl);
    fgSoln.runtime = duration.count();
    fgSoln.solverType = 0;
    fgSoln.numTimeSteps = numTimeSteps;
    // fgSoln.orderingType = ordering

    // Calculate Cost of Factor Graph
    double cost_fg = 0.;

    std::cout << "Calculating Total Cost of FG" << std::endl;

    for(int i1 = 0; i1 < numTimeSteps; i1++) {

        // Add Costs For States
        for(int i2=0; i2 < numNodes; i2++) {

            fgSoln.states(i2, i1) = ( gtsam::Vector(2) << resultLQR.at<VectorXd>(X[i2][i1])(0),
                                                          resultLQR.at<VectorXd>(X[i2][i1])(1) ).finished();

            if (i1 < numTimeSteps - 1){
                cost_fg += pow(fgSoln.states(i2,i1)(0),2.)*qNode(0) + pow(fgSoln.states(i2,i1)(1),2.)*qNode(1);
            } else {
                cost_fg += pow(fgSoln.states(i2,i1)(0),2.)*qfNode(0) + pow(fgSoln.states(i2,i1)(1),2.)*qfNode(1);
            }

        }

        // Add Costs For Controls
        if(i1 < numTimeSteps - 1) {
            for(int i4=0; i4<numControl; i4++) {

                fgSoln.controls(i4, i1) = resultLQR.at<VectorXd>(U[i4][i1])(0);
                cost_fg += pow(fgSoln.controls(i4,i1),2.)*R_full(i4,i4);
            } 
        }
    }     

    std::cout << "Total Cost for FG: " << cost_fg << std::endl;
    fgSoln.cost = cost_fg;

    return fgSoln;
}

// Ricatti Equation To Solve LQR Problem
SolverOutput FGSolver::DARESolve()
{
    std::cout << "Entered DARE Solve" << std::endl;

    vector<MatrixXd> P_val(numTimeSteps, Qf_full);
    MatrixXd X_lqr = MatrixXd::Zero(2*numNodes, numTimeSteps);
    MatrixXd U_lqr = MatrixXd::Zero(numControl, numTimeSteps-1);
    MatrixXd cost_lqr = MatrixXd::Zero(1,1);

    SolverOutput lqrSoln(numNodes, numTimeSteps, numControl);
    lqrSoln.solverType = 3;

    std::cout << "Starting Timer" << std::endl;

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    // Backward Pass With Ricatti
    for (int i=numTimeSteps-1; i>0; i--) {
        P_val[i-1] = Q_full + A_full.transpose()*P_val[i]*A_full - 
        A_full.transpose()*P_val[i]*B_full*(R_full + B_full.transpose()*
        P_val[i]*B_full).colPivHouseholderQr().solve(B_full.transpose()*P_val[i]*A_full);
    }

    // Forward Pass
    X_lqr.col(0) = X_init;

    for(int i=0; i<numTimeSteps-1; i++) {

        // std::cout << "Check a" << std::endl;
        // MatrixXd k_mat = -1.0*(R_full + B_full.transpose()*P_val[i]*B_full).colPivHouseholderQr().solve(B_full.transpose()*P_val[i]*A_full);

        // std::cout << "Check b" << std::endl;
        // U_lqr.col(i) = k_mat * X_lqr.col(i);

        // std::cout << "Check b2" << std::endl;
        // X_lqr.col(i+1) = A_full * X_lqr.col(i) + B_full * U_lqr.col(i);

        // std::cout << "Check c" << std::endl; 
        // cost_lqr += X_lqr.col(i).transpose() * Q_full * X_lqr.col(i);
        // cost_lqr += U_lqr.col(i).transpose() * R_full * U_lqr.col(i);

		MatrixXd k_mat = -1.0*(R_full + B_full.transpose()*P_val[i]*B_full)
			.colPivHouseholderQr().solve(B_full.transpose()*P_val[i]*A_full);

        // std::cout << k_mat << std::endl;

        // std::cout << "\n" << std::endl;

        // std::cout << X_lqr.col(i) << std::endl;

		U_lqr.col(i) = k_mat * X_lqr.col(i);
		X_lqr.col(i+1) = A_full * X_lqr.col(i) + B_full * U_lqr.col(i);

	 	cost_lqr += X_lqr.col(i).transpose() * Q_full * X_lqr.col(i);
	 	cost_lqr += U_lqr.col(i).transpose() * R_full * U_lqr.col(i);


    }

    cost_lqr += X_lqr.col(numTimeSteps-1).transpose() * Qf_full * X_lqr.col(numTimeSteps-1);

    end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsedTime = end - start;

    std::cout << "The DARE Solution Took " << elapsedTime.count() << " seconds\n" << std::endl;

    std::cout << "DARE Cost: " << cost_lqr(0,0) << std::endl;

    lqrSoln.cost = cost_lqr(0,0);
    lqrSoln.controls = U_lqr;
    lqrSoln.runtime = elapsedTime.count();

    std::cout << "before matrix out" << std::endl;

    matrixToOutput(lqrSoln, X_lqr);

    std::cout << "after matrix out" << std::endl;

    return lqrSoln;
}

// Data Manipulation
void FGSolver::matrixToOutput(SolverOutput& soln, const MatrixXd& X_mat)
{
    for (int i1=0; i1 < numTimeSteps; i1++){

        for (int i2=0; i2 < numNodes; i2++){
            soln.states(i2, i1) = (gtsam::Vector(1) << X_mat(i2, i1)).finished();
        }
    }

    return;

}