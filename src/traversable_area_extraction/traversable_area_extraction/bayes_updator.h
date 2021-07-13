#ifndef BAYES_UPDATOR_H
#define BAYES_UPDATOR_H
namespace traversable_area_extraction{


inline double BayesUpdator(double value_update, double value_observe){
  double tem_value = 0;
  if (value_update <=0 ) {
    tem_value = value_observe;
    return tem_value;
  }
  double tem_odds  = (value_update*value_observe)/(1-value_observe-value_update +value_observe*value_update);
  tem_value = tem_odds/(1+tem_odds);
  if(tem_value>0.9) tem_value = 0.9;
  if(tem_value<0.2) tem_value = 0.2;
  return tem_value;


  }
}
#endif
