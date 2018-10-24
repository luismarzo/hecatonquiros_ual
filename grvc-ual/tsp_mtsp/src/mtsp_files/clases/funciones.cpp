
#include "funciones.h"

using namespace std;
 using std::vector;



    void split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
}


vector<string> split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}

/*
 void pedir_pantalla_f(int &cont_repeticion,int &A, int &B, vector<vector<float>> &puntos,
 vector<float> &punto_final){
 std::string input_file = "";
 std::ifstream myfile;
 int coordenadas,n_puntos;
  float pto;

  string line;
    vector <string> words;
    vector <float> p_inicial;
    vector <vector<float>> p;

  myfile.open(input_file);
  if (myfile.is_open()){

	// COORDENADAS
   getline (myfile,line) ;
		words = split(line,' ');
		coordenadas=stoi(words[1]);
      //cout << coordenadas<< '\n';
      words.clear();
    //cout<<endl;

    // PUNTOS_INTERMEDIOS
    getline (myfile,line) ;
		words = split(line,' ');
		n_puntos=stoi(words[1]);
      //cout << n_puntos<< '\n';
      n_puntos=n_puntos+1;
      words.clear();
    //cout<<endl;

    puntos.resize(n_puntos);


    // PUNTO_INICIAL
    getline (myfile,line) ;
		words = split(line,' ');
	puntos[0].resize(coordenadas);
	for (int j=0;j<puntos[0].size();j++){
	puntos[0][j]=stof(words[j+1]);
	//cout << puntos[0][j]<< '\n';
	}
      words.clear();
    //cout<<endl;

    // PUNTO_FINAL
    getline (myfile,line) ;
		words = split(line,' ');
	punto_final.resize(coordenadas);
	for (int j=0;j<punto_final.size();j++){
	punto_final[j]=stof(words[j+1]);
	//cout << punto_final[j]<< '\n';
	}
      words.clear();
    //cout<<endl;

  // PUNTOS TRAYECTORIA
  getline (myfile,line);

  for(int i=0;i<(n_puntos-1);i++){
  getline (myfile,line) ;
		words = split(line,' ');
	puntos[i+1].resize(coordenadas);
	for (int j=0;j<puntos[i+1].size();j++){
	puntos[i+1][j]=stof(words[j]);
	//cout << puntos[i+1][j]<< '\n';
	}
      words.clear();
    //cout<<endl;
  }


  // NUMERO DE REPETICIONES
   getline (myfile,line) ;
		words = split(line,' ');
		cont_repeticion=stoi(words[1]);
       //cout << cont_repeticion<< '\n';
      words.clear();
    //cout<<endl;

    myfile.close();
  }

  else{ cout << "Unable to open file";
}



   B=coordenadas; A=n_puntos;

 }
*/

std::vector<float> cut_matrix (const vector<vector<float> > &matriz,float fila,float columna){

	if (columna==-1){
		vector<float> resultado;
		resultado.resize(matriz[0].size());
		for (int i = 0; i < matriz[0].size(); i++) {
		resultado[i] = matriz[fila][i];
		}
			return resultado;

	}

	if(fila==-1){
		vector<float> resultado;
		resultado.resize(matriz.size());
		for (int z = 0; z < matriz.size(); z++) {
		resultado[z] = matriz[z][columna];
		}
			return resultado;

	}
}

float minimo(std::vector<float> temp){
	float y;
	float v[temp.size()]; float *p;

	for(int i=0;i<temp.size();i++){

		v[i]=temp[i];
	}

	p=std::min_element(v,v+temp.size()) ;
	y=*p;

	return y;

}

float norma(std::vector<float> u){
	float accum = 0,norm;
    for (int i = 0; i < u.size(); i++) {
        accum += u[i] * u[i];
    }
    return norm = sqrt(accum);
}

vector<float> resta(vector<float> r1,vector<float> r2){
	vector<float> res;
	res.resize(r1.size());

	for(int i=0; i<r1.size();i++){
		res[i]=r1[i]-r2[i];
	}
	return res;
}

vector<float> ordena_vector(vector<float> v){
	vector<float> v_,v_aux;
	int lon=v.size(),flag,contt,cont_;
	v_.resize(v.size());
	v_aux.resize(v.size());
	contt=0; ; cont_=0;

	for(int j=0;j<lon;j++){
		flag=0;

		for(int i=0;i<v.size();i++){

			if(minimo(v)==v[i] && flag==0){
				v_[cont_]=v[i];
				flag=1; v[i]=0; cont_=cont_+1;

			}

		}

		for(int i=0;i<v.size();i++){

		if(v[i]==0){}
		else{
			v_aux.resize(v.size()-1);
			v_aux[contt]=v[i];
		  contt=1+contt;

		}
		}
		v.clear();
		v.resize(v_aux.size());
		v=v_aux;
		contt=0;
	}
			return v_;


}


vector<float> reordena_vector2(vector<float> v_original, vector<float> v_ordenado, vector<float> v_a_cambiar){
	vector<float> v_cambiado;int flag=0;
	v_cambiado.resize(v_original.size());

	for(int i=0; i<v_original.size(); i++){
	for(int j=0; j<v_ordenado.size(); j++){

		if(v_original[j]==v_ordenado[i] && flag==0){

			v_cambiado[i]=v_a_cambiar[j];
			v_original[j]=-1; flag=1;
		}

	}
	flag=0;
	}
	return v_cambiado;

	}




nodo ordena_nodo(nodo p){
	vector<float> auxiliar;
	auxiliar.resize(p.camino.size());

	auxiliar=ordena_vector(p.camino);
	p.punto_sig=reordena_vector2(p.camino,auxiliar,p.punto_sig);
	p.camino_activo=reordena_vector2(p.camino,auxiliar,p.camino_activo);

	p.camino=auxiliar;
	return p;
}

vector<float> refresca_auxiliar_f( vector<float> entrada){
	vector<float> indices_aux, indices=entrada,salida;
	for(int x=0;x<indices.size();x++){
			if(indices[x]==0){}
		else{
			indices_aux.push_back (indices[x]);
		}
	}
	salida=indices_aux;
	return salida;
}





int devuelve_activo(std::vector<float> v){

	int res, flag=0;
	for(int i=0;i<v.size();i++){
		if(flag==0 && v[i]==1){
			flag=1; res=1;
		}
		if((v.size()-1)==i && flag==0){
			res=0;
		}
	}
	return res;
	}


int componente_rep(vector<float> v,int numero){
	int res=0;
	for(int i=0;i<v.size();i++){
		if(numero==v[i]){
			res=1;}
	}// fin del "for"
	return res;
} // fin de "componente_rep"


float es_ultimo(int &B, struct nodo n,const vector<vector<float> > &dots, vector<float> punto_final){

	vector<float> c,r1;
	float distancia;
	r1.resize(B); c.resize(1); //numero de coordenadas


	if(n.punto_sig.size()==0){
		c[0]=n.punto_act;
		r1=cut_matrix(dots,c[0],-1);
		distancia=norma(resta(r1,punto_final));
	}
	else{
		distancia=0;
	}
	return distancia;
}





struct s_dev_hijo devuelve_hijo (int &B, struct nodo padre,
const vector<vector<float> > &dots,float dis,int counter){
	struct s_dev_hijo salida;
    vector<vector<float> > aux;
    int flag; float dist,distancia;
   	vector<float> mul,c,r1,r2;

   	//EMPIEZAN RESIZES
   	c.resize(2);
	//r1.resize(B); r2.resize(B); //numero de coordenadas

	padre.camino.resize(padre.punto_sig.size());
	padre.camino_activo.resize(padre.punto_sig.size());
	padre.escoge.resize(padre.punto_sig.size());

   	//ACABAN RESIZES

   	salida.p=padre;

	salida.h.indicador=counter; counter=counter+1;
	salida.h.nivel=1+salida.p.nivel;
	salida.h.definido=0;



	if(salida.p.definido==1){}
	else{
		for(int i=0 ; i < salida.p.punto_sig.size() ; i++){
			c[0]=salida.p.punto_act;
			 c[1]=salida.p.punto_sig[i];
			r1=cut_matrix(dots,c[0],-1);
	        r2=cut_matrix(dots,c[1],-1);
			 distancia=norma(resta(r1,r2));
			salida.p.camino[i]=distancia;
			salida.p.camino_activo[i]=1;

		}//FIN DEL FOR
		salida.p.vivo=1;
		salida.p=ordena_nodo(salida.p);

	}

	aux.resize(salida.p.camino_activo.size(), vector<float>(salida.p.camino_activo.size()));

	for(int i=0 ; i < salida.p.camino_activo.size() ; i++){
	for(int j=0 ; j < salida.p.camino_activo.size() ; j++){
	 aux[i][j]=1;
	}}

	salida.p.escoge=cut_matrix(aux,0,-1);

	flag=0;

		for(int i=0 ; i < salida.p.camino_activo.size() ; i++){
		if(flag==0 && salida.p.camino_activo[i]==1){
			flag=1;
			salida.p.camino_activo[i]=0;
			salida.p.escoge[i]=0;
			dist=salida.p.camino[i];
			salida.h.distancia_recorrida=dist+dis;
			salida.h.punto_act=salida.p.punto_sig[i];

		}
		}

		if(devuelve_activo(salida.p.camino_activo)==0){
			salida.p.vivo=0;
		}

		salida.p.definido=1;

	//MULTIPLICO ELEMENTO A ELEMENTO--->BEGIN
	//se hace el resize
	   	mul.resize(salida.p.escoge.size());

	for(int i=0 ; i < salida.p.escoge.size() ; i++){
		mul[i]=salida.p.punto_sig[i]*salida.p.escoge[i];
	}


	salida.h.punto_sig=refresca_auxiliar_f(mul);

	// REPASAR ESTAS DOS ÚLTIMAS LÍNEAS
	salida.h.recorrido=salida.p.recorrido;
    salida.h.recorrido.push_back (salida.h.punto_act);

    // DEVUELVO LA SALIDA
    salida.contador=counter;

    return salida;

}


struct s_desg_nodo desglosa_nodo (int &B,struct nodo padre,
const vector<vector<float> > &puntos2,float dis,int contador_dh){

struct s_dev_hijo salida2;
struct s_desg_nodo devuelto;
int salir,contador;

salir=0; contador=1;



 while(salir==0){

	salida2=devuelve_hijo(B,padre,puntos2,dis,contador_dh);

contador=contador+1;
devuelto.a.push_back (salida2.h);


if(salida2.p.vivo==0){
	salir=1;
}

padre=salida2.p;
contador_dh=salida2.contador;

 }

devuelto.p=padre;
devuelto.contador=salida2.contador;

return devuelto;

}


void salida_desglosa_nodo(s_desg_nodo salida_dn){

	ROS_INFO("DESGLOSE, Y'ALL: ");
  ROS_INFO("\n");

ROS_INFO("CONTADOR: ");
            cout << salida_dn.contador << " ";
  ROS_INFO("\n");


	 ROS_INFO("NIVEL HIJOS: ");
for (int i = 0; i < salida_dn.a.size(); i++) {
            cout << salida_dn.a[i].nivel << " ";
        }
  ROS_INFO("\n");

  ROS_INFO("DEFINIDO HIJOS: ");
for (int i = 0; i < salida_dn.a.size(); i++) {
            cout << salida_dn.a[i].definido << " ";
        }
  ROS_INFO("\n");

  ROS_INFO("DIS_RECORR. HIJOS: ");
for (int i = 0; i < salida_dn.a.size(); i++) {
            cout << salida_dn.a[i].distancia_recorrida << " ";
        }
  ROS_INFO("\n");

   ROS_INFO("PUNTO ACT. HIJOS: ");
for (int i = 0; i < salida_dn.a.size(); i++) {
            cout << salida_dn.a[i].punto_act << " ";
        }
  ROS_INFO("\n");

     ROS_INFO("INDICADOR HIJOS: ");
for (int i = 0; i < salida_dn.a.size(); i++) {
            cout << salida_dn.a[i].indicador << " ";
        }
  ROS_INFO("\n");

     ROS_INFO("RECORRIDO HIJOS: ");
for (int i = 0; i < salida_dn.a.size(); i++) {
	cout << "HIJO " << i << ": " ;
	for (int j = 0; j < salida_dn.a[i].recorrido.size(); j++) {
            cout << salida_dn.a[i].recorrido[j] << " ";
		}
		ROS_INFO("\n");
        }
  ROS_INFO("\n");

     ROS_INFO("PUNTO SIG. HIJOS: ");
for (int i = 0; i < salida_dn.a.size(); i++) {
	cout << "HIJO " << i << ": " ;
	for (int j = 0; j < salida_dn.a[i].punto_sig.size(); j++) {
            cout << salida_dn.a[i].punto_sig[j] << " ";
		}
		ROS_INFO("\n");
        }
  ROS_INFO("\n");

  ROS_INFO("¿CUANTOS PUNTOS SIGUIENTES HAY?: ");
  cout << salida_dn.a[0].punto_sig.size() << " ";
    ROS_INFO("\n");


}


void repeticion(int &B, int &salir,nodo &nodo_1,
vector<nodo> &nodo_desglosable,int &INDICE,float &distancia,
vector< vector<float> > &puntos2, vector <float> &punto_final,
s_desg_nodo &salida_dn, float &counter, vector<nodo> &v_n_maduros,
int &tope, vector<float> &nodo_pre, vector<float> &c, vector<float> &r1,
float &auxx, vector<float> &nodo_final, vector<float> &v,
int &flag){

// EMPIEZA REPETICION

 // VARIABLES
 salir=0;
 nodo_1=nodo_desglosable[INDICE];  // en matlab pone nodo_desglosable[INDICE];
 distancia=nodo_1.distancia_recorrida;


 if(es_ultimo(B,nodo_desglosable[INDICE],puntos2,punto_final)>0){
 // no desgloso el nodo si está en el último nivel que quiero
}

else{

	while(salir==0){ // COMIENZO DEL WHILE
	// desgloso los nodos del nivel inferior al que considero
	// y lo defino completamente
		salida_dn=desglosa_nodo(B,nodo_1,puntos2,distancia,counter);
		counter=salida_dn.contador;
		v_n_maduros.push_back(salida_dn.p);
		nodo_1=salida_dn.a[0];
		distancia=salida_dn.a[0].distancia_recorrida;

		for (int j=0;j<salida_dn.a.size();j++){
		nodo_desglosable.push_back(salida_dn.a[j]);}

	if (salida_dn.a[0].nivel>=(tope+1)){

	for(int ii=0;ii<salida_dn.a.size();ii++){
		nodo_pre.push_back(salida_dn.a[ii].distancia_recorrida);


		c[0]=salida_dn.a[ii].punto_act;

		r1=cut_matrix(puntos2,c[0],-1);
		auxx=norma(resta(r1,punto_final));

		nodo_final.push_back((salida_dn.a[ii].distancia_recorrida)+auxx);
	} // FIN DEL FOR
		salir=1;
	} // FIN DEL IF

	} // FIN DEL WHILE



} // FIN DEL ELSE

// SECCION DE "REPETICION" ENCARGADA DE REFRESCAR ÍNDICE ENTRE ITERACIONES
	// Recorremos los nodos desarrollados, "v_n_maduros"


	for (int i=0; i<v_n_maduros.size();i++){

		if((v_n_maduros[i].indicador==0) || ((v_n_maduros[i].indicador)-(v_n_maduros[i-1].indicador)<0) || (componente_rep(v,v_n_maduros[i].indicador))){
		}
		else{
			v.push_back(v_n_maduros[i].indicador);
		}// fin del "IF"
	} // fin del "for"

	flag=0;
	for (int i=0; i<nodo_desglosable.size();i++){

		if((nodo_desglosable[i].indicador>0) & (componente_rep(v,nodo_desglosable[i].indicador)==0) & (flag==0) & (nodo_desglosable[i].punto_sig.size()>0)){

		INDICE=i;
		nodo_desglosable[i].indicador=0; flag=1;

		}// FIN DEL "IF"

	}// FIN DEL "FOR"


// FIN "REPETICION"


}

void imprimir_resultados(int cont_repeticion,int A, int B,
  vector<vector<float>> puntos_recorrido,
  vector<vector<float>> puntos, vector<float> MINIMO,
  vector<nodo> aux, int vital){

	cout << "DATOS DE ENTRADA " << endl;
  cout << "Espacio " << B << "D " << endl;
  cout << "Número de repeticiones: " << cont_repeticion << endl;


  cout << "Ciudad inicio= [ " ;
	for (int i = 0; i < B; i++) {
		if(i==(B-1)){
			cout << puntos_recorrido[0][i] << " ] ";
		}
		else{
            cout << puntos_recorrido[0][i] << ", ";
        }
	}
  cout << endl;

    cout << "Ciudad destino= [ " ;
	for (int i = 0; i < B; i++) {
		if(i==(B-1)){
			cout << puntos_recorrido[A][i] << " ] ";
		}
		else{
            cout << puntos_recorrido[A][i] << ", ";
        }
	}
  cout << endl;

  cout << "Ciudades intermedias ( " << (A-1) << " )"<< endl;
	for (int i = 1; i < A; i++) {
		for (int j=0; j<B;j++){

		if(j==0){
			cout << "  " << i << ": [ " << puntos[i][j] << ", ";
		}
		else {
			if(j==(B-1)){
            cout << puntos[i][j] << " ] " << endl;
			}
			else{
			cout << puntos[i][j] << ", ";
			}
		}}
	}


  cout << endl;

  cout << "DATOS DE SALIDA " << endl;
  cout << "Distancia recorrida: " << MINIMO[0] << " uds" << endl;
  cout << "Recorrido " << endl;
       for (int i = 0; i < (A+1); i++) {
		for (int j=0; j<B;j++){

		if(j==0){
			if((i==0) || (i==A)){
			cout <<"      [ " << puntos_recorrido[i][j] << ", ";
		    }
		    else{
			cout <<"  " << aux[0].recorrido[i] << ":  [ " << puntos_recorrido[i][j] << ", ";
			}
		}

		else {
			if(j==(B-1)){

			if (i==0){
            cout << puntos_recorrido[i][j] << " ]   INICIO  (PUNTO BLANCO)" << endl;
		    }
		    if (i==A){
            cout << puntos_recorrido[i][j] << " ]   FINAL  (PUNTO ROJO)" << endl;
		    }
		    if (i>0 && i<A){
            cout << puntos_recorrido[i][j] << " ]   (PUNTO VERDE)" << endl;
		    }

            if(i<A){
				if(B==3){
            cout <<"           | " << endl;
            cout <<"           |   ( " << norma(resta(puntos_recorrido[i],puntos_recorrido[i+1])) << " ) uds" << endl;
            cout <<"           | " << endl;
            cout <<"           V " << endl;}
                if(B==2){
            cout <<"         | " << endl;
            cout <<"         |   ( " << norma(resta(puntos_recorrido[i],puntos_recorrido[i+1])) << " ) uds" << endl;
            cout <<"         | " << endl;
            cout <<"         V " << endl;}
		    }

			}
			else{
			cout << puntos_recorrido[i][j] << ", ";
			}
		}}
	}
   cout << "Óptimo alcanzado en iteración: " << (vital+1) << endl;
   cout << endl;

}
