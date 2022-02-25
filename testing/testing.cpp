    for (auto i = linconstraints.begin(); i != linconstraints.end(); i++) {

      if (i->restlength > i->original_length*2) { //split constraint, 

        float alpha = i->alpha/2;
        float restlength = i->restlength/2.f
        auto pos1 = i->pos1;
        auto pos2 = i->pos2;
        glm::vec2 posc = (i->pos1->position + i->pos2->position)/2;
        positions.push_back(posc);
        auto posc_i = --positions.end();
        i->pos1 = posc_i;
        i->alpha = alpha;
        i->restlength = restlength ;
        linconstraints.insert(i, LinConstraint(pos1, posc_i, alpha, restlength));

        auto behind_iter = i-2;
        auto front_iter = i+1;
        bool do_behind = behind_iter.has_value();
        bool do_front = front_iter.has_value();

        // now handle the angle constraints

        auto ang1 = AngularConstraint(i->begin_ang->pos1, b->pos2, (i-1)->pos1, angalpha);
        auto ang2 = AngularConstraint((i-1)->pos1, i->pos2, i->pos1, angalpha);
        auto ang3 = AngularConstraint(i->pos1, i->end_ang->pos2, i->pos2, angalpha);
  


        //remove the orignal angular constraints
        if (do_behind) angconstraints.remove(i->begin_ang.value());
        if (do_front) angconstraints.remove(i->end_ang.value());

        //and replace them
        if (do_behind) {
          angconstrains.emplace_back(i->begin_ang->pos1, b->pos2, (i-1)->pos1, angalpha);
          behind_iter->ang1 = angconstraints.end - 1;
          if (i != linconstraints.begin()) 
            (i-2)->ang2 = angconstraints.end - 1;
        }
        
        angconstraints.push_back(ang2);
        (i-1)->ang2 = angconstraints.end - 1;
        i->ang1 = angconstraints.end - 1;

        angconstraints.push_back(ang3);
        i->ang2 = angconstraints.end - 1;
        (i+1)->ang1 = angconstraints.end - 1;
      }

    }










        