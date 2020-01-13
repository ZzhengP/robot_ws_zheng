#include "solver/lpsolve.h"



lpSolver::lpSolver(int Nv, int Nc):Nv_(Nv),Nc_(Nc){

    this->structureVar_["n1"] = 1;
    this->structureVar_["n2"] = 2;
    this->structureVar_["n3"] = 3;
    this->structureVar_["b"] = 4;
    this->structureVar_["d"] = 5;
    this->structureVarBnds_.insert(std::make_pair(-1,1));
    this->structureVarBnds_.insert(std::make_pair(-1,1));
    this->structureVarBnds_.insert(std::make_pair(-1,1));
    this->structureVarBnds_.insert(std::make_pair(-2,2));
    this->structureVarBnds_.insert(std::make_pair(-1,1));

    this->lp_ = glp_create_prob();
    glp_set_prob_name(lp_,"Separating Plane");
    glp_set_obj_dir(lp_,GLP_MIN);

    std::map<std::string, int>::iterator it = this->structureVar_.begin();
    std::map<int,int>::iterator it2 = this->structureVarBnds_.begin();
    glp_add_cols(lp_,Nv_);

    // Add variable_columns n,d,r
//   for(int i(0) ; i< Nv_; i++){
//        std::cout <<"char array name  " << it->first <<std::endl;

//        char char_array[it->first.length() + 1];
//        strcpy(char_array, it->first.c_str());
//        glp_set_col_name(lp_,i+1,char_array);
//        glp_set_col_bnds(lp_,i+1,GLP_DB,it2->first, it2->second);
//        it++;
//        it2++;
//    }
    glp_set_col_name(lp_,1,"n1");
    glp_set_col_name(lp_,2,"n2");
    glp_set_col_name(lp_,3,"n3");
    glp_set_col_name(lp_,4,"b");
    glp_set_col_name(lp_,5,"d");
    glp_set_col_bnds(lp_,1,GLP_DB,-1, 1);
    glp_set_col_bnds(lp_,2,GLP_DB,-1, 1);
    glp_set_col_bnds(lp_,3,GLP_DB,-1, 1);
    glp_set_col_bnds(lp_,4,GLP_FR,0, 2);
    glp_set_col_bnds(lp_,5,GLP_FR,-1, 1);

    // Initialize auxiliary variable an then fill when we get data from robot
    for (int i(0);i<Nc_;i++){
        this->auxiliaryVar_.insert(std::make_pair("aux"+std::to_string(i+1),i+1));
    }
    glp_add_rows(lp_,Nc_);
    A_.resize(Nc_,Nv_);
    A_.setZero();
    Solution_.resize(5);
    Solution_.setZero();
    std::cout <<"---------------------------------------------" << std::endl;
    std::cout << BOLD(FRED("lps solver is successively created: "))  <<'\n';
                for (int j(0);j<Nv_;j++){
                 std::cout << "colum name " << glp_get_col_name(lp_,j+1) <<'\n';
                }

    std::cout <<"---------------------------------------------" << std::endl;

};


void lpSolver::setCost(){
    glp_set_obj_coef(lp_, 1, 0);
    glp_set_obj_coef(lp_, 2, 0);
    glp_set_obj_coef(lp_, 3, 0);
    glp_set_obj_coef(lp_, 4, 0);
    glp_set_obj_coef(lp_, 5, 1);
 }

void lpSolver::setRowConstraint(){
    std::map<std::string,int>::iterator it3 = this->auxiliaryVar_.begin();
    while (it3 != this->auxiliaryVar_.end()){
        int n = it3->first.length();
//        std::cout <<"char aux array name  " << it3->first <<std::endl;

        char char_array[n + 1];
        strcpy(char_array, it3->first.c_str());
        glp_set_row_name(lp_,it3->second,char_array);
        it3++;
    }

    for (int i(1);i<Nc_;i++){

        glp_set_row_bnds(lp_,i,GLP_UP,0,0);

    }

    glp_set_row_bnds(lp_,Nc_,GLP_DB,0.99,1);


}

void lpSolver::setEqualityConstraintMatrix(){
    // Retrieve the index of matrix A,
    l_ = A_.rows(); col_ = A_.cols();

    for (int i(1);i<l_+1;i++){
        for (int j(1); j<col_+1; j++){
               ia_[(i-1)*col_+j] = i ;
               ja_[(i-1)*col_+j] = j;
               ar_[(i-1)*col_+j] = A_(i-1,j-1) ;
         }
    }
}

void lpSolver::setEqualityConstraintMatrix(Eigen::MatrixXd A){
          // Retrieve the index of matrix A,
         l_ = A.rows(); col_ = A.cols();
         for (int i(1);i<l_+1;i++){
           for (int j(1); j<col_+1; j++){
                ia_[(i-1)*col_+j] = i ;
                ja_[(i-1)*col_+j] = j;
                ar_[(i-1)*col_+j] = A(i-1,j-1);
               }
          }
   }

void lpSolver::setConstraintMatrix(const Eigen::MatrixXd & robotPartielVertices,
                                   const Eigen::MatrixXd & obsPartielVertices, const Eigen::MatrixXd & dataPlanePrecedent){


    A_.block(0,0,1,5) << robotPartielVertices.block(0,0,3,1).transpose(),-1,-1;
    A_.block(1,0,1,5) << robotPartielVertices.block(0,1,3,1).transpose(),-1,-1;
    A_.block(2,0,1,5) << robotPartielVertices.block(0,2,3,1).transpose(),-1,-1;
    A_.block(3,0,1,5) << robotPartielVertices.block(0,3,3,1).transpose(),-1,-1;
//    A_.block(2,0,1,5) << robotPartielVertices.block(0,2,3,1).transpose(),-1,0;
//    A_.block(3,0,1,5) << robotPartielVertices.block(0,3,3,1).transpose(),-1,0;
//    A_.block(2,0,1,5) << -obsPartielVertices.block(0,0,3,1).transpose(),1,0;
    A_.block(4,0,1,5) << -obsPartielVertices.block(0,1,3,1).transpose(),1,-1;
    A_.block(5,0,1,5) << dataPlanePrecedent.transpose(),0,0;

}

void lpSolver::print(){
    std::cout <<"---------------------------------------------" << std::endl;
    std::cout << BOLD(FRED("lps solver information: "))  <<'\n';
                std::cout << glp_get_prob_name(lp_) << " is created "<<"\n";
                 std::cout <<"Problem's variable name are: \n" ;
                for (int j(0);j<Nv_;j++){
                 std::cout << " " << glp_get_col_name(lp_,j+1) <<'\n';
                }
                std::cout <<"Problem's objective are: \n" ;
                for (int j(0);j<Nv_;j++){
                 std::cout << " " << glp_get_obj_coef(lp_,j+1) <<'\n';
                }
                  std::cout << BOLD(FRED("row constraint: \n "))  <<'\n';
                for (int j(0); j<Nc_;j++){
                    std::cout<< " row lb constraint  name: " << glp_get_row_name(lp_,j+1) <<"\n";
                    std::cout<<"auxialiary lb constraint " << j+1 << " is: " << glp_get_row_lb(lp_,j+1)<<"\n" ;
                    std::cout<<"auxialiary ub constraint " << j+1 << " is: " << glp_get_row_ub(lp_,j+1) <<"\n" ;
                }
                 std::cout << BOLD(FRED("column constraint: \n "))  <<'\n';

                 for (int k(0); k<Nv_;k++){
                    std::cout<< " column lb constraint  name: " << glp_get_col_name(lp_,k+1) <<"\n";
                    std::cout<<"column lb constraint " << k+1 << " is: " << glp_get_col_lb(lp_,k+1)<<"\n" ;
                    std::cout<<"column ub constraint " << k+1 << " is: " << glp_get_col_ub(lp_,k+1) <<"\n" ;

                }

    std::cout <<"---------------------------------------------" << std::endl;

}
