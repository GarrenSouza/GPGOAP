/*
Copyright 2012 Abraham T. Stolk

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.
 */
#include "goap.h" // for planner interface.
#include "astar.h" // for A* search over worldstate space.


#include <string.h>
#include <stdio.h>
#include <time.h>

int main(int argc, char* argv[]) {
    static actionplanner_t ap;
    goap_actionplanner_clear(&ap);
    goap_set_pre(&ap, "atordoar", "inimigo_visivel", true);
    goap_set_pre(&ap, "atordoar", "estou_em_batalha", true);

    goap_set_pre(&ap, "deslocar_4>3", "inimigo_visivel", true);
    goap_set_pre(&ap, "deslocar_4>3", "estou_em_batalha", false);
    goap_set_pre(&ap, "deslocar_4>3", "distancia_inimigo_4", true);
    goap_set_pst(&ap, "deslocar_4>3", "distancia_inimigo_3", true);
    goap_set_pst(&ap, "deslocar_4>3", "distancia_inimigo_4", false);
    goap_set_pst(&ap, "deslocar_4>3", "estou_em_batalha", true);

    goap_set_pre(&ap, "deslocar_3>2", "eu_vivo", true);
    goap_set_pre(&ap, "deslocar_3>2", "estou_em_batalha", true);
    goap_set_pre(&ap, "deslocar_3>2", "distancia_inimigo_3", true);
    goap_set_pst(&ap, "deslocar_3>2", "distancia_inimigo_2", true);
    goap_set_pst(&ap, "deslocar_3>2", "distancia_inimigo_3", false);

    goap_set_pre(&ap, "deslocar_2>1", "inimigo_visivel", true);
    goap_set_pre(&ap, "deslocar_2>1", "estou_em_batalha", true);
    goap_set_pre(&ap, "deslocar_2>1", "distancia_inimigo_2", true);
    goap_set_pst(&ap, "deslocar_2>1", "distancia_inimigo_1", true);
    goap_set_pst(&ap, "deslocar_2>1", "distancia_inimigo_2", false);

    goap_set_pre(&ap, "deslocar_1>2", "inimigo_visivel", true);
    goap_set_pre(&ap, "deslocar_1>2", "estou_em_batalha", true);
    goap_set_pre(&ap, "deslocar_1>2", "distancia_inimigo_1", true);
    goap_set_pst(&ap, "deslocar_1>2", "distancia_inimigo_2", true);
    goap_set_pst(&ap, "deslocar_1>2", "distancia_inimigo_1", false);

    goap_set_pre(&ap, "deslocar_2>3", "inimigo_visivel", true);
    goap_set_pre(&ap, "deslocar_2>3", "estou_em_batalha", true);
    goap_set_pre(&ap, "deslocar_2>3", "distancia_inimigo_2", true);
    goap_set_pst(&ap, "deslocar_2>3", "distancia_inimigo_3", true);
    goap_set_pst(&ap, "deslocar_2>3", "distancia_inimigo_2", false);

    goap_set_pre(&ap, "deslocar_3>4", "inimigo_visivel", true);
    goap_set_pre(&ap, "deslocar_3>4", "estou_em_batalha", true);
    goap_set_pre(&ap, "deslocar_3>4", "distancia_inimigo_3", true);
    goap_set_pst(&ap, "deslocar_3>4", "distancia_inimigo_4", true);
    goap_set_pst(&ap, "deslocar_3>4", "distancia_inimigo_3", false);
    goap_set_pst(&ap, "deslocar_3>4", "estou_em_batalha", false);

    goap_set_pre(&ap, "buscar_abrigo", "inimigo_visivel", true);
    goap_set_pre(&ap, "buscar_abrigo", "eu_vivo", true);
    goap_set_pre(&ap, "buscar_abrigo", "abrigo_proximo", false);
    goap_set_pst(&ap, "buscar_abrigo", "abrigo_proximo", true);

    goap_set_pre(&ap, "curar", "inimigo_visivel", true);
    goap_set_pre(&ap, "curar", "eu_vivo", true);

    goap_set_pre(&ap, "suicidio", "eu_vivo", true);
    goap_set_pre(&ap, "suicidio", "inimigo_visivel", true);
    goap_set_pre(&ap, "suicidio", "amigos_em_volta", false);
    goap_set_pre(&ap, "suicidio", "inimigo_vivo", true);
    goap_set_pre(&ap, "suicidio", "distancia_inimigo_1", true);
    goap_set_pst(&ap, "suicidio", "eu_vivo", false);
    goap_set_pst(&ap, "suicidio", "inimigo_vivo", false);
    goap_set_cost(&ap, "suicidio", 5);

    goap_set_pre(&ap, "curar", "estou_em_batalha", false);
    goap_set_pre(&ap, "curar", "minha_vida_<64", true);
    goap_set_pre(&ap, "curar", "distancia_inimigo_3", true);
    goap_set_pre(&ap, "curar", "amigos_em_volta", true);
    goap_set_pre(&ap, "curar", "sob_abrigo", true);
    goap_set_pst(&ap, "curar", "minha_vida_<64", false);

    goap_set_pre(&ap, "esconder", "inimigo_visivel", true);
    goap_set_pst(&ap, "esconder", "estou_em_batalha", true);
    goap_set_pre(&ap, "esconder", "eu_vivo", true);
    goap_set_pre(&ap, "esconder", "abrigo_proximo", true);
    goap_set_pst(&ap, "esconder", "estou_em_batalha", false);
    goap_set_pst(&ap, "esconder", "sob_abrigo", true);

    goap_set_pre(&ap, "retornar_a_batalha", "estou_em_batalha", false);
    goap_set_pst(&ap, "retornar_a_batalha", "estou_em_batalha", true);

    goap_set_pre(&ap, "procurar", "inimigo_visivel", false);
    goap_set_pst(&ap, "procurar", "inimigo_visivel", true);
    char desc[ 4096 ];
    goap_description(&ap, desc, sizeof (desc));
    //LOGI( "%s", desc );


    worldstate_t fr;
    goap_worldstate_clear(&fr);
    goap_worldstate_set(&ap, &fr, "inimigo_visivel", true);
    goap_worldstate_set(&ap, &fr, "eu_vivo", true);
    goap_worldstate_set(&ap, &fr, "minha_vida_<64", true);
    goap_worldstate_set(&ap, &fr, "estou_em_batalha", true);
    goap_worldstate_set(&ap, &fr, "amigos_em_volta", true);
    goap_worldstate_set(&ap, &fr, "inimigo_vivo", true);
    goap_worldstate_set(&ap, &fr, "vida_inimigo_alta", false);
    goap_worldstate_set(&ap, &fr, "vida_inimigo_media", true);
    goap_worldstate_set(&ap, &fr, "vida_inimigo_baixa", false);
    goap_worldstate_set(&ap, &fr, "qnt_memoria_3", true);
    goap_worldstate_set(&ap, &fr, "qnt_memoria_2", false);
    goap_worldstate_set(&ap, &fr, "qnt_memoria_1", false);
    goap_worldstate_set(&ap, &fr, "distancia_inimigo_4", false);
    goap_worldstate_set(&ap, &fr, "distancia_inimigo_3", false);
    goap_worldstate_set(&ap, &fr, "distancia_inimigo_2", false);
    goap_worldstate_set(&ap, &fr, "distancia_inimigo_1", true);
    goap_worldstate_set(&ap, &fr, "abrigo_proximo", false);
    goap_worldstate_set(&ap, &fr, "sob_abrigo", false);
    goap_worldstate_set(&ap, &fr, "minha_vida<8", true);
    goap_worldstate_set(&ap, &fr, "inimigo_atordoado", false);
    goap_worldstate_set(&ap, &fr, "estou_atordoado", false);

    //goap_worldstate_set( &ap, &fr, "distancia_inimigo_1", true );

    worldstate_t goal;
    goap_worldstate_clear(&goal);
    goap_worldstate_set(&ap, &goal, "minha_vida_<64", false);
    goap_worldstate_set(&ap, &goal, "distancia_inimigo_3", false);
    goap_worldstate_set(&ap, &goal, "estou_em_batalha", true);
    //goap_worldstate_set( &ap, &goal, "inimigo_vivo", false);
    goap_worldstate_set(&ap, &goal, "eu_vivo", true);
    worldstate_t states[16];
    const char* plan[16];
    int plansz = 16;
    const int plancost = astar_plan(&ap, fr, goal, plan, states, &plansz);
    LOGI("custo = %d", plancost);
    goap_worldstate_description(&ap, &fr, desc, sizeof ( desc));
    //LOGI( "%-23s%s", "", desc );

    for (int i = 0; i < plansz && i < 16; ++i) {
        goap_worldstate_description(&ap, states + i, desc, sizeof ( desc));
        LOGI("%d: %-20s \n %s \n", i, plan[i], desc);
    }
    return plancost;
}


