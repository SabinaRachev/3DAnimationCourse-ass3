#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include <igl\vertex_triangle_adjacency.h>
#include "Eigen/dense"
#include <igl/circulation.h>
#include <functional>



SandBox::SandBox()
{
	

}

void SandBox::Init(const std::string &config)
{
	std::string item_name;
	std::ifstream nameFileout;
	doubleVariable = 0;
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file "<<config << std::endl;
	}
	else
	{
		
		while (nameFileout >> item_name)
		{
			std::cout << "openning " << item_name << std::endl;
			load_mesh_from_file(item_name);
			
			parents.push_back(-1);
			data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
			data().show_overlay_depth = false;
			data().point_size = 10;
			data().line_width = 2;
			data().set_visible(false, 1);

			
		}
		nameFileout.close();
	}
	MyTranslate(Eigen::Vector3d(0, 0, -1), true);
	
	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));

}




SandBox::~SandBox()
{
}
void SandBox::loadCylindersAndSphere(int numberOfCylinder) {
	lastLinkId = numberOfCylinder>0? numberOfCylinder:-1;
	firstLinkId = numberOfCylinder > 0 ? 1 : -1;
	numberOfLinks = numberOfCylinder;
	int num = numberOfCylinder + 1;
	parents.resize(num);
	children.resize(num);
	std::ifstream file("configuration.txt");
	std::string line;
	//load sphere 
	if (!std::getline(file, line)) {
		line = "C:/Users/sabin/Downloads/EngineForAnimationCourse/tutorial/data/sphere.obj";
	}
	load_mesh_from_file(line);
	addShape(0, 0);
	data_list[0].MyTranslate(Eigen::Vector3d(5, 0, 0), true);
	parents[0] = -1;
	if (!std::getline(file, line)) {
		line = "C:/Users/sabin/Downloads/EngineForAnimationCourse/tutorial/data/zcylinder.obj";
	}
	for (int i = 1; i < num; i++) {
		load_mesh_from_file(line);
		make_zaxis(i);
		if (i == 1) 
			parents[i] = -1;
		else
			parents[i] = i - 1; //set parent
		if (i + 1 < num)
			children[i] = i + 1;
		else 
			children[i] = -1;
	}
	float z_top = data_list[numberOfCylinder].V.colwise().maxCoeff()[2];
	link_length = z_top * 2;

	MyTranslate(Eigen::Vector3d(0, -2, -9),true);
	data_list[firstLinkId].MyTranslate(Eigen::Vector3d(0, 0, -link_length),true);
	targetId = 0;
}
void SandBox::loadAnotherCylinder(std::string fname) {
	load_mesh_from_file(fname);
	int id = data_list.size() - 1;
	trees.resize(id+1);
	parents.resize(id + 1);
	children.resize(id + 1);
	make_zaxis(id);
	parents[id]=lastLinkId;
	children[lastLinkId] = id;
	children[id] = -1;
	lastLinkId = id;
	data_list.back().set_visible(false, 1);
	data_list.back().set_visible(true, 2);
	data_list.back().show_faces = 3;
	selected_data_index = id;
	numberOfLinks++;
}
void SandBox::addShape(int id,int zpoint) {

	//draw axis 
	if (id != 0) {
		addAxisToObject(id);
	}
	data_list[id].add_points(Eigen::RowVector3d(0, 0, zpoint), Eigen::RowVector3d(0, 0, 1));
	data_list[id].show_overlay_depth = true;
	data_list[id].point_size = 10;
	data_list[id].line_width = 2;
	data_list[id].set_visible(false, 1);

}
void SandBox::make_zaxis(int id){
	Eigen::MatrixXd& V = data_list[id].V;
	Eigen::Vector3d max = V.colwise().maxCoeff();
	Eigen::Vector3d min = V.colwise().minCoeff();
	float max_z = max[2];
	float min_z = min[2];
	//initialize
	data_list[id].SetCenterOfRotation(Eigen::Vector3d(V.colwise().mean()[0], V.colwise().mean()[1], min_z));
	data_list[id].MyTranslate(Eigen::Vector3d(0, 0, 2 * max_z), true);
	addShape(id,-0.8);

	
}
void SandBox::addAxisToObject(int id) {
	float lengthOfAxis = data_list[id].V.colwise().maxCoeff()[2] - data_list[id].V.colwise().minCoeff()[2];

	Eigen::Vector3d m = data().V.colwise().minCoeff();
	Eigen::MatrixXd centerOfAxis(1, 3);
	centerOfAxis << data_list[id].V.colwise().mean()[0], data_list[id].V.colwise().mean()[1], m(2);

	Eigen::MatrixXd axisBounding(6, 3);
	axisBounding << centerOfAxis(0) + lengthOfAxis, centerOfAxis(1), centerOfAxis(2),
		centerOfAxis(0) - lengthOfAxis, centerOfAxis(1), centerOfAxis(2),
		//Handle Y
		centerOfAxis(0), centerOfAxis(1) + lengthOfAxis, centerOfAxis(2),
		centerOfAxis(0), centerOfAxis(1) - lengthOfAxis, centerOfAxis(2),
		//Handle Z
		centerOfAxis(0), centerOfAxis(1), centerOfAxis(2) + lengthOfAxis,
		centerOfAxis(0), centerOfAxis(1), centerOfAxis(2) - lengthOfAxis;

     data_list[id].add_points(centerOfAxis, Eigen::RowVector3d(0, 0, 0));

	data_list[id].add_edges(axisBounding.row(0),  axisBounding.row(1), Eigen::RowVector3d(255, 0, 1));
	data_list[id].add_edges(axisBounding.row(2), axisBounding.row(3), Eigen::RowVector3d(255, 1, 0));
	data_list[id].add_edges(axisBounding.row(4), axisBounding.row(5), Eigen::RowVector3d(255, 0, 0));

}

void SandBox::ikCyclicCoordinateDecentMethod(){
	if (shouldAnimateCCD) {
		Eigen::Vector3d target_des = data_list[targetId].MakeTransd().col(3).head(3);
		Eigen::Vector3d first_link_pos = ikGetPosition(firstLinkId, 0);
		if ((target_des - first_link_pos).norm() > link_length * numberOfLinks) {
			std::cout << "cannot reach" << std::endl;
			shouldAnimateCCD = false;
			return;
		}
		int currLink = lastLinkId;
		while (currLink != -1) {
			Eigen::Vector3d r = ikGetPosition(currLink, 0);
			Eigen::Vector3d e = ikGetPosition(lastLinkId, link_length);
			Eigen::Vector3d rd = target_des - r;
			Eigen::Vector3d re = e - r;
			Eigen::Vector3d normal = re.normalized().cross(rd.normalized());//returns the plane normal
			double distance = (target_des - e).norm();
			if (distance < 0.1) {
				std::cout << "distance: " << distance << std::endl;
				fix_rotate();
				shouldAnimateCCD = false;
				return;
			}
			double dot = rd.normalized().dot(re.normalized());
			//check that it is beetween -1 to 1
			if (dot > 1)
				dot = 1;
			if (dot < -1)
				dot = -1;
			double angle = acosf(dot) / 10;
			Eigen::Vector3d rotationVec = (CalcParentsTrans(currLink) * data_list[currLink].MakeTransd()).block<3, 3>(0, 0).inverse() * normal;
			int parent = parents[currLink];
			if (ikSolverConstrainDegree && parent != -1) {
				data_list[currLink].MyRotate(rotationVec, angle);
				e = ikGetPosition(lastLinkId, link_length); //get new position after rotation
				re = e - r;
				Eigen::Vector3d r_parent = ikGetPosition(parent, 0);
				rd = r_parent - r;
				//find angle between parent and link
				double constarin = 0.5235987756;
				double parentDot = rd.normalized().dot(re.normalized());//get dot 
				if (parentDot > 1)
					parentDot = 1;
				if (parentDot < -1)
					parentDot = 1;
				double parentAngle = acos(parentDot);
				data_list[currLink].MyRotate(rotationVec, -angle);//rotate back 
				if (parentAngle < constarin) {//fix angle
					angle = angle - (constarin - parentAngle);
				}

			}
			data_list[currLink].MyRotate(rotationVec, angle);
			currLink = parents[currLink];

		}
	}

}




Eigen::Vector3d SandBox::ikGetPosition(int id, double length){
	Eigen::Vector3d CrotationCurr = data_list[id].getCenterOfRotation();
	Eigen::Vector4d rCenter(CrotationCurr[0], CrotationCurr[1], CrotationCurr[2] + length, 1);
	Eigen::Vector3d r = (CalcParentsTrans(id) * data_list[id].MakeTransd() * rCenter).head<3>();
	return r;
} 

void SandBox::ikFabrik() {
	if (shouldAnimateFabrik) {
		std::vector<Eigen::Vector3d> p; //joint positions
		p.resize(data_list.size() + 1);
		Eigen::Vector3d t = data_list[targetId].MakeTransd().col(3).head(3);
		Eigen::Vector3d root = ikGetPosition(firstLinkId, 0);;

		//Set disjoint positions
		//p1 is first disjoin
		int curr = lastLinkId;
		while (curr != -1) {
			p[curr] = ikGetPosition(curr, 0);
			curr = parents[curr];
		}
		p[lastLinkId + 1] = ikGetPosition(lastLinkId, link_length);
		std::vector<double> riArr;
		std::vector<double> lambdaIArr;

		riArr.resize(data_list.size() + 1);
		lambdaIArr.resize(data_list.size() + 1);

		if ((t - root).norm() > link_length * numberOfLinks) {
			/*1.5
			If Unreachable
			*/
			std::cout << "cannot reach" << std::endl;
			shouldAnimateFabrik = false;
			return;
		}
		else
		{
			/*
			*1.10
			Target Is Reachable
			*/
			Eigen::Vector3d b = p[firstLinkId]; //the target is reachable; thus set as b the initial position of joint p0

			//Check wether the distance between the end effector Pn and the target t is greater then a tolerance

			Eigen::Vector3d endEffector = p[lastLinkId + 1];
			float tolerance = 0.1;

			float diffA = (endEffector - t).norm();
			if (diffA < tolerance) {
				std::cout << "distance : " << diffA << "\n" << std::endl;
				shouldAnimateFabrik = false;
				return;
			}
			while (diffA > tolerance) {
				//	1.19 Stage1 Forward Reaching
				p[lastLinkId + 1] = t;
				int parent = lastLinkId;
				int child = lastLinkId + 1;
				while (parent != -1) {
					//1.23 Find the distance ri between the new joint position pi+1 and the joint pi
					riArr[parent] = (p[child] - p[parent]).norm();
					lambdaIArr[parent] = link_length / riArr[parent];
					p[parent] = (1 - lambdaIArr[parent]) * p[child] + lambdaIArr[parent] * p[parent]; //1.27
					child = parent;
					parent = parents[parent];

				}
				/*
				Stage 2
				Backward reaching
				1.29*/
				//Set the root p0 its initial position*/
				p[firstLinkId] = b;
				parent = firstLinkId;
				child = children[firstLinkId];
				while (child != -1) {
					//1.33 Find the distance ri between the new joint position pi and the joint pi+1
					riArr[parent] = (p[child] - p[parent]).norm();
					lambdaIArr[parent] = link_length / riArr[parent];
					p[child] = (1 - lambdaIArr[parent]) * p[parent] + lambdaIArr[parent] * p[child]; //1.27
					parent = child;
					child = children[child];
				}

				riArr[lastLinkId] = (p[lastLinkId + 1] - p[lastLinkId]).norm();
				lambdaIArr[lastLinkId] = link_length / riArr[lastLinkId];
				p[lastLinkId + 1] = (1 - lambdaIArr[lastLinkId]) * p[lastLinkId] + lambdaIArr[lastLinkId] * p[lastLinkId + 1]; //1.27
				diffA = (p[lastLinkId + 1] - t).norm();
			}
			//rotate
			int currLink = firstLinkId;
			int target_id = children[firstLinkId];
			while (target_id != -1) {
				ikSolverHelper(currLink, p[target_id]);
				currLink = target_id;
				target_id = children[target_id];
			}
			ikSolverHelper(lastLinkId, p[lastLinkId + 1]);
			double distance = (t - ikGetPosition(lastLinkId, link_length)).norm();
			if (distance < tolerance) {
				fix_rotate();
				shouldAnimateFabrik = false;
				std::cout << "distance: " << distance << std::endl;

			}
		}
	}
}






void SandBox::ikSolverHelper(int id, Eigen::Vector3d t){
	Eigen::Vector3d r = ikGetPosition(id, 0);
	Eigen::Vector3d e = ikGetPosition(id, link_length);
	Eigen::Vector3d rd = t - r;
	Eigen::Vector3d re = e - r;
	Eigen::Vector3d normal = re.normalized().cross(rd.normalized());
	double dot = rd.normalized().dot(re.normalized());//get dot 
	if (dot > 1)
		dot = 1;
	if (dot < -1)
		dot = 1;
	double angle = acos(dot)/10 ;
	Eigen::Vector3d rotationVec = (CalcParentsTrans(id) * data_list[id].MakeTransd()).block<3, 3>(0, 0).inverse() * normal;
	int parent = parents[id];
	//bonus
	if (ikSolverConstrainDegree && parent != -1) {
		data_list[id].MyRotate(rotationVec, angle);
		e= ikGetPosition(id, link_length); //get new position after rotation
		 re = e - r;
		 Eigen::Vector3d r_parent= ikGetPosition(parent, 0);
		 rd = r_parent - r;
		 //find angle between parent and link
		 double constarin = 0.5235987756;
		 double parentDot = rd.normalized().dot(re.normalized());//get dot 
		 if (parentDot > 1)
			 parentDot = 1;
		 if (parentDot < -1)
			 parentDot = 1;
		 double parentAngle = acos(parentDot);
		 data_list[id].MyRotate(rotationVec, -angle);//rotate back 
		 if (parentAngle < constarin) {//fix angle
			 angle = angle - (constarin - parentAngle);
		 }
	}
	data_list[id].MyRotate(rotationVec, angle);
}

void SandBox::fix_rotate(){
	Eigen::Vector3d Z(0, 0, 1);
	int currLink = firstLinkId;
	while (currLink != -1) {
		Eigen::Matrix3d R = data_list[currLink].GetRotation();
		Eigen::Vector3d ea = R.eulerAngles(2, 0, 2);//get the rotation angles
		double angleZ = ea[2];
		data(currLink).MyRotate(Z, -angleZ);
		currLink = children[currLink];
		if (currLink != -1)
			data(currLink).RotateInSystem(Z, angleZ);
	}
}


void SandBox::move2Objects(){
	data_list[0].MyTranslate(Eigen::Vector3d(1,0,0), true);
	data_list[1].MyTranslate(Eigen::Vector3d(-1, 0, 0), true);

}

void SandBox::initData()
{
	int size = data_list.size();
	//reasize all data structures
	resizeDataStructers(size);
	//initialize data structures
	for (int i = 0; i < size; i++) {
		initData(i);
	}

}
void SandBox::resizeDataStructers(int size) {
	//reasize all data structures
	E.resize(size);
	EMAP.resize(size);
	EF.resize(size);
	Q.resize(size);
	EI.resize(size);
	C.resize(size);
	Qit.resize(size);
	Qmatrix.resize(size);
	num_collapsed.resize(size);
	trees.resize(size);
}
void SandBox::initData(int i) {	
	Eigen::MatrixXd V = data_list[i].V;  //vertice matrix
	Eigen::MatrixXi F = data_list[i].F; //faces matrix
	igl::AABB<Eigen::MatrixXd, 3> objectTree;
	objectTree.init(V, F);
	trees[i]=objectTree;
	drawBox(trees[i].m_box, 0,i);
	igl::edge_flaps(F, E[i], EMAP[i], EF[i], EI[i]);//init data_structures
	C[i].resize(E[i].rows(), V.cols());
	Qit[i].resize(E[i].rows()); //number of edges 
	caculateQMatrix(V, F, i);
	Q[i].clear();
	num_collapsed[i] = 0;
	//caculate egdes cost
	for (int j = 0; j < E[i].rows(); j++)
		caculateCostAndPlacment(i, j, V);

}


void SandBox::caculateQMatrix(Eigen::MatrixXd& V, Eigen::MatrixXi& F, int index){
	std::vector<std::vector<int> > VF;// vertex to faces
	std::vector<std::vector<int> > VFi;//not used
	int n = V.rows();
	Qmatrix[index].resize(n);
	igl::vertex_triangle_adjacency(n, F, VF, VFi);
	Eigen::MatrixXd F_normals = data_list[index].F_normals;
	

	for (int i = 0; i < n; i++) {
		//initialize 
		Qmatrix[index][i] = Eigen::Matrix4d::Zero();

		//caculate vertex  Q matrix 
		for (int j = 0; j < VF[i].size(); j++) {
			Eigen::Vector3d normal = F_normals.row(VF[i][j]).normalized();//get face normal
			// the equation is ax+by+cz+d=0
			Eigen::Matrix4d curr;
			double a = normal[0];
			double b = normal[1];
			double c = normal[2];
			double d = V.row(i) * normal;
			d *= -1;
			curr.row(0) = Eigen::Vector4d(a*a, a*b, a*c, a*d);
			curr.row(1) = Eigen::Vector4d(a*b, b*b, b*c, b*d);
			curr.row(2) = Eigen::Vector4d(a*c, b*c, c*c, c*d);
			curr.row(3) = Eigen::Vector4d(a*d,b*d, c*d, d*d);
			Qmatrix[index][i] += curr;
		}

	}
}
// compute cost and potential placement and place in queue
void SandBox::caculateCostAndPlacment(int index, int edge, Eigen::MatrixXd& V)
{
	//vertexes of the edge
	int v1 = E[index](edge, 0);
	int v2 = E[index](edge, 1);

	Eigen::Matrix4d Qedge= Qmatrix[index][v1] + Qmatrix[index][v2];

	Eigen::Matrix4d Qposition = Qedge; //we will use this to find v` position
	Qposition.row(3) = Eigen::Vector4d(0, 0, 0, 1);
	Eigen::Vector4d vposition;
	double cost;
	bool isInversable;
	Qposition.computeInverseWithCheck(Qposition, isInversable);
	if (isInversable) {
		vposition = Qposition * (Eigen::Vector4d(0, 0, 0, 1));
		cost = vposition.transpose() * Qedge * vposition;
	}
	else {
		//find min error from v1 v2 v1+v2/2
		Eigen::Vector4d v1p;
		v1p<< V.row(v1), 1;;
		double cost1 = v1p.transpose() * Qedge * v1p;

		Eigen::Vector4d v2p;
		v1p << V.row(v2), 1;;
		double cost2 = v2p.transpose() * Qedge * v2p;

		Eigen::Vector4d v12p;
		v1p << ((V.row(v1)+ V.row(v2))/2), 1;;
		double cost3 = v12p.transpose() * Qedge * v12p;
		if (cost1 < cost2 && cost1 < cost3) {
			vposition = v1p;
			cost = cost1;
		}
		else if (cost2 < cost1 && cost2 < cost3) {
			vposition = v2p;
			cost = cost2;
		}
		else {
			vposition = v12p;
			cost = cost3;

		}
	}
	Eigen::Vector3d pos;
	pos[0] = vposition[0];
	pos[1] = vposition[1];
	pos[2] = vposition[2];
	C[index].row(edge) = pos;
	Qit[index][edge] = Q[index].insert(std::pair<double, int>(cost, edge)).first;

}

void SandBox::simplification(){
	int id = data().id;
	Eigen::MatrixXd& V = data().V;  //vertice matrix
	Eigen::MatrixXi& F = data().F; //faces matrix
	bool something_collapsed = false;
	// collapse edge
	const int max_iter = std::ceil(0.05 * Q[id].size());//collapse 5%
	for (int j = 0; j < max_iter; j++)
	{
		if (!collapse_edge(V,F,id)){
			break;
		}
		something_collapsed = true;
		num_collapsed[id]++;
	}

	if (something_collapsed)
	{   
		//data().clear();
		data().set_mesh(V, F);
		data().set_face_based(true);
		data().dirty = 157;

	}
}
bool SandBox::collapse_edge(Eigen::MatrixXd& V, Eigen::MatrixXi& F, int id){
	PriorityQueue&  curr_Q=Q[id];
	std::vector<PriorityQueue::iterator >& curr_Qit = Qit[id];
	int e1, e2, f1, f2; //be used in the igl collapse_edge function
	if (curr_Q.empty())
	{
		// no edges to collapse
		return false;
	}
	std::pair<double, int> pair = *(curr_Q.begin());
	if (pair.first == std::numeric_limits<double>::infinity())
	{
		// min cost edge is infinite cost
		return false;
	}
	curr_Q.erase(curr_Q.begin()); //delete from the queue
	int e = pair.second; //the lowest cost edge in the queue
	//the 2 vertix of the edge
	int v1 = E[id].row(e)[0];
	int v2 = E[id].row(e)[1];

	curr_Qit[e] = curr_Q.end();

	//get the  list of faces around the end point the edge
	std::vector<int> N = igl::circulation(e, true, EMAP[id], EF[id], EI[id]);
	std::vector<int> Nd = igl::circulation(e, false, EMAP[id], EF[id], EI[id]);
	N.insert(N.begin(), Nd.begin(), Nd.end());

	//collapse the edage
	bool is_collapsed = igl::collapse_edge(e, C[id].row(e), V, F, E[id], EMAP[id], EF[id], EI[id], e1, e2, f1, f2);
	if(is_collapsed){


		// Erase the two, other collapsed edges
		curr_Q.erase(curr_Qit[e1]);
		curr_Qit[e1] = curr_Q.end();
		curr_Q.erase(curr_Qit[e2]);
		curr_Qit[e2] = curr_Q.end();

		//update the Q matrix for the 2 veterixes we collapsed 
		Qmatrix[id][v1] = Qmatrix[id][v1] + Qmatrix[id][v2];
		Qmatrix[id][v2] = Qmatrix[id][v1] + Qmatrix[id][v2];

		Eigen::VectorXd newPosition;
		// update local neighbors
		// loop over original face neighbors
		for (auto n : N)
		{
			if (F(n, 0) != IGL_COLLAPSE_EDGE_NULL ||
				F(n, 1) != IGL_COLLAPSE_EDGE_NULL ||
				F(n, 2) != IGL_COLLAPSE_EDGE_NULL)
			{
				for (int v = 0; v < 3; v++)
				{
					// get edge id
					 const  int ei = EMAP[id](v * F.rows() + n);
					// erase old entry
					curr_Q.erase(curr_Qit[ei]);
					// compute cost and potential placement and place in queue
				  	caculateCostAndPlacment(id, ei, V);
					newPosition = C[id].row(ei);
				}
			}
		}
		std::cout << "edge " << e << ",cost " << pair.first << ",new position (" << newPosition[0] << ","
			<< newPosition[1] << "," << newPosition[2] << ")" << std::endl;
	}
	else
	{
		// reinsert with infinite weight (the provided cost function must **not**
		// have given this un-collapsable edge inf cost already)
		pair.first = std::numeric_limits<double>::infinity();
		curr_Qit[e] = curr_Q.insert(pair).first;
	}
	return is_collapsed;

}

void SandBox::drawBox(Eigen::AlignedBox<double, 3>& box, int color,int id) {
	Eigen::RowVector3d colorVec;
	if (color == 1) {
		colorVec = Eigen::RowVector3d(255, 0, 0);
	}
	else if (color == 2) {
		colorVec = Eigen::RowVector3d(0, 0, 255);
	}
	else colorVec = Eigen::RowVector3d(0, 255, 0);
	Eigen::RowVector3d BottomRightCeil = box.corner(box.BottomRightCeil);
	Eigen::RowVector3d BottomRightFloor = box.corner(box.BottomRightFloor);
	Eigen::RowVector3d BottomLeftCeil = box.corner(box.BottomLeftCeil);
	Eigen::RowVector3d BottomLeftFloor = box.corner(box.BottomLeftFloor);
	Eigen::RowVector3d TopRightCeil = box.corner(box.TopRightCeil);
	Eigen::RowVector3d TopRightFloor = box.corner(box.TopRightFloor);
	Eigen::RowVector3d TopLeftCeil = box.corner(box.TopLeftCeil);
	Eigen::RowVector3d TopLeftFloor = box.corner(box.TopLeftFloor);
	data_list[id].add_edges(BottomLeftCeil, BottomRightCeil, colorVec);
	data_list[id].add_edges(BottomLeftCeil, BottomLeftFloor, colorVec);
	data_list[id].add_edges(BottomRightCeil, BottomRightFloor, colorVec);
	data_list[id].add_edges(BottomLeftFloor, BottomRightFloor, colorVec);
	data_list[id].add_edges(TopLeftCeil, TopRightCeil, colorVec);
	data_list[id].add_edges(TopRightCeil, TopRightFloor, colorVec);
	data_list[id].add_edges(TopLeftCeil, TopLeftFloor, colorVec);
	data_list[id].add_edges(TopLeftFloor, TopRightFloor, colorVec);
	data_list[id].add_edges(TopLeftCeil, BottomLeftCeil, colorVec);
	data_list[id].add_edges(TopRightFloor, BottomRightFloor, colorVec);
	data_list[id].add_edges(TopRightCeil, BottomRightCeil, colorVec);
	data_list[id].add_edges(TopLeftFloor, BottomLeftFloor, colorVec);
	data_list[id].point_size = 10;
	data_list[id].line_width = 3;
}

bool SandBox::thereIsCollision(igl::AABB<Eigen::MatrixXd, 3>* treeA, igl::AABB<Eigen::MatrixXd, 3>* treeB,int id,int other_id){
	//base cases
	if (treeA == nullptr || treeB == nullptr)
		return false;
	
	if (!boxesIntersect(treeA->m_box, treeB->m_box, other_id)) {
		return false;
	}
	if (treeA->is_leaf() && treeB->is_leaf()) {
		//if the boxes intersect than draw the  boxes
		if (boxesIntersect(treeA->m_box, treeB->m_box, other_id)) {
			std::cout << "collapse" << std::endl;
			std::cout << id << std::endl;
			std::cout << other_id << std::endl;
			drawBox(treeA->m_box, 2, id);
			drawBox(treeB->m_box, 1, other_id);
			return true;
		}
		else {
			return false;
		}

	}
	 if (treeA->is_leaf() && !treeB->is_leaf()) {
		 return thereIsCollision(treeA, treeB-> m_right,id, other_id) ||
			 thereIsCollision(treeA, treeB->m_left,id,  other_id);
	}
	 if (!treeA->is_leaf() && treeB->is_leaf()) {
		 return thereIsCollision(treeA->m_right, treeB,id, other_id) ||
			  thereIsCollision(treeA->m_left, treeB,id, other_id);
	 }

	//recursively check for intersactions case
    return thereIsCollision(treeA->m_left, treeA->m_left,id, other_id) ||
		   thereIsCollision(treeA->m_left, treeB->m_right,id, other_id) ||
		   thereIsCollision(treeA->m_right, treeB->m_left,id, other_id) ||
		   thereIsCollision(treeA->m_right, treeB->m_right,id, other_id);
}


bool SandBox::boxesIntersect(Eigen::AlignedBox<double, 3>& boxA, Eigen::AlignedBox<double, 3>& boxB, int other_id){
	// matrix A
	Eigen::Matrix3d A = data_list[selected_data_index].GetRotation().cast<double>();
	Eigen::Vector3d A0 = A.col(0);
	Eigen::Vector3d A1 = A.col(1);
	Eigen::Vector3d A2 = A.col(2);

	// matrix B
	Eigen::Matrix3d B = data_list[other_id].GetRotation().cast<double>();
	Eigen::Vector3d B0 = B.col(0);
	Eigen::Vector3d B1 = B.col(1);
	Eigen::Vector3d B2 = B.col(2);
	//C=A^T*B
	Eigen::Matrix3d C = A.transpose() * B;
	//get the lengths of the sides of the bounding box
	Eigen::Vector3d a = boxA.sizes();
	Eigen::Vector3d b = boxB.sizes();
	a = a / 2;
	b = b / 2;
	//build matrix D
	Eigen::Vector4d CenterA = Eigen::Vector4d(boxA.center()[0], boxA.center()[1], boxA.center()[2], 1);
	Eigen::Vector4d CenterB = Eigen::Vector4d(boxB.center()[0], boxB.center()[1], boxB.center()[2], 1);
	Eigen::Vector4d D4d = data_list[other_id].MakeTransd().cast<double>() * CenterB - data_list[selected_data_index].MakeTransd().cast<double>() * CenterA;
	Eigen::Vector3d D = D4d.head(3);
	//check the 15 conditions
	//check A conditions
	if (a(0) + (b(0) * abs(C.row(0)(0)) + b(1) * abs(C.row(0)(1)) + b(2) * abs(C.row(0)(2))) < abs(A0.transpose() * D))
		return false;
	if (a(1) + (b(0) * abs(C.row(1)(0)) + b(1) * abs(C.row(1)(1)) + b(2) * abs(C.row(1)(2))) < abs(A1.transpose() * D))
		return false;
	if (a(2) + (b(0) * abs(C.row(2)(0)) + b(1) * abs(C.row(2)(1)) + b(2) * abs(C.row(2)(2))) < abs(A2.transpose() * D))
		return false;
	//check B conditions
	if (b(0) + (a(0) * abs(C.row(0)(0)) + a(1) * abs(C.row(1)(0)) + a(2) * abs(C.row(2)(0))) < abs(B0.transpose() * D))
		return false;
	if (b(1) + (a(0) * abs(C.row(0)(1)) + a(1) * abs(C.row(1)(1)) + a(2) * abs(C.row(2)(1))) < abs(B1.transpose() * D))
		return false;
	if (b(2) + (a(0) * abs(C.row(0)(2)) + a(1) * abs(C.row(1)(2)) + a(2) * abs(C.row(2)(2))) < abs(B2.transpose() * D))
		return false;
	//check A0 
	double R = C.row(1)(0) * A2.transpose() * D;
	R-=C.row(2)(0) * A1.transpose() * D;
	if (a(1) * abs(C.row(2)(0)) + a(2) * abs(C.row(1)(0)) + b(1) * abs(C.row(0)(2))+ b(2) * abs(C.row(0)(1)) < abs(R))
		return false;

	R = C.row(1)(1) * A2.transpose() * D;
	R -= C.row(2)(1) * A1.transpose() * D;
	if (a(1) * abs(C.row(2)(1)) + a(2) * abs(C.row(1)(1)) + b(0) * abs(C.row(0)(2)) + b(2) * abs(C.row(0)(0)) < abs(R))
		return false;

	R = C.row(1)(2) * A2.transpose() * D;
	R -= C.row(2)(2) * A1.transpose() * D;
	if (a(1) * abs(C.row(2)(2)) + a(2) * abs(C.row(1)(2)) + b(0) * abs(C.row(0)(1)) + b(1) * abs(C.row(0)(0)) < abs(R))
		return false;
	//check A1 conditions

	 R = C.row(2)(0) * A0.transpose() * D;
	R -= C.row(0)(0) * A2.transpose() * D;
	if (a(0) * abs(C.row(2)(0)) + a(2) * abs(C.row(0)(0)) + b(1) * abs(C.row(1)(2)) + b(2) * abs(C.row(1)(1)) < abs(R))
		return false;

	R = C.row(2)(1) * A0.transpose() * D;
	R -= C.row(0)(1) * A2.transpose() * D;
	if (a(0) * abs(C.row(2)(1)) + a(2) * abs(C.row(0)(1)) + b(0) * abs(C.row(1)(2)) + b(2) * abs(C.row(1)(0)) < abs(R))
		return false;

	R = C.row(2)(2) * A0.transpose() * D;
	R -= C.row(0)(2) * A2.transpose() * D;
	if (a(0) * abs(C.row(2)(2)) + a(2) * abs(C.row(0)(2)) + b(0) * abs(C.row(1)(1)) + b(1) * abs(C.row(1)(0)) < abs(R))
		return false;
	//check A2 conditions

	 R = C.row(0)(0) * A1.transpose() * D;
	R -= C.row(1)(0) * A0.transpose() * D;
	if (a(0) * abs(C.row(1)(0)) + a(1) * abs(C.row(0)(0)) + b(1) * abs(C.row(2)(2)) + b(2) * abs(C.row(2)(1)) < abs(R))
		return false;

	R = C.row(0)(1) * A1.transpose() * D;
	R -= C.row(1)(1) * A0.transpose() * D;
	if (a(0) * abs(C.row(1)(1)) + a(1) * abs(C.row(0)(1)) + b(0) * abs(C.row(2)(2)) + b(2) * abs(C.row(2)(0)) < abs(R))
		return false;
	R = C.row(0)(2) * A1.transpose() * D;
	R -= C.row(1)(2) * A0.transpose() * D;
	if (a(0) * abs(C.row(1)(2)) + a(1) * abs(C.row(0)(2)) + b(0) * abs(C.row(2)(1)) + b(1) * abs(C.row(2)(0)) < abs(R))
		return false;
	
	return true;
}






/*
Added this data structure to hold our KdTrees
*/






void SandBox::Animate(){
	if (shouldAnimate) {
		data_list[selected_data_index].MyTranslate(direction, true);
		int size = data_list.size();
			for (int i = 0; i < size; i++) {
				if (i != selected_data_index) {
					if (thereIsCollision(&trees[selected_data_index], &trees[i], selected_data_index, i)) {
						shouldAnimate = false;
						break;
					}
          }		
		}
	}
	else if (shouldAnimateCCD) {
		ikCyclicCoordinateDecentMethod();
	}
	else if (shouldAnimateFabrik) {
		ikFabrik();
	}

}




