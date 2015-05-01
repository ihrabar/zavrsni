#include <agv_control/vehInfo.h>
#include <nav_msgs/GridCells.h>

using namespace std;

void sendRemovalRequest(string vehicleName);
void publishGridCells(nav_msgs::GridCells gCells);

vector<vector<int> > ColWithTimeMy;
vector<vector<int> > ColWithoutTimeMy;

vector<vector<int> > ColWithTimeOther;
vector<vector<int> > ColWithoutTimeOther;

bool stop = false;

// Provjera vremenskog preklapanja zauzetosti čelije kroz koju prolaze oba vozila
bool overlap(float ti1, float to1, float ti2, float to2)
{
	if (to1 < ti2 || to2 < ti1)
		return false;

	return true;
}

// Provjera kolizije
void Collision(const agv_control::vehInfo& myInfo, const vector<agv_control::vehInfo>& otherVehInfo, int mapWidth, int mapHeight)
{
	ColWithTimeMy.clear();
	ColWithoutTimeMy.clear();

	for (unsigned int i = 0; i < (otherVehInfo.size() + 1); i++)
	{
		ColWithTimeMy.push_back(vector<int>());
		ColWithoutTimeMy.push_back(vector<int>());
		ColWithTimeOther.push_back(vector<int>());
		ColWithoutTimeOther.push_back(vector<int>());
	}

	// Dvodimenzionalno polje u koje se za svaku čeliju kroz koju vozilo prolazi..
	// ..upisuje indeks segmenta putanje na kojem se vozilo nalazi u toj čeliji
	vector<int> li [mapWidth][mapHeight];

	// Dvodimenzionalno polje u koje se upisuje vrijeme ulaska (ti) vozila u čelije kroz koje vozilo prolazi
	float* ti[mapHeight];
	for(int i = 0; i < mapHeight; i++)
		ti[i] = new float[mapWidth];

	// Dvodimenzionalno polje u koje se upisuje vrijeme izlaska (to) vozila iz čelija kroz koje vozilo prolazi
	float* to[mapHeight];
	for(int i = 0; i < mapHeight; i++)
		to[i] = new float[mapWidth];

	float deltaTime = 0; //10 * r.GetTimeToFinish();  ****************************** OVO DORADITI !!!!!!! ***************************
	float max;
	int tmpX;
	int tmpY;

	// Pretraživanje po svim segmentima putanje..
	for (unsigned int i = 0; i < myInfo.grid.xGrid.size(); i++)
	{
		max = 0.0;

		// Pretraživanje po svim čelijama kroz koje vozilo prolazi na pojedinom segmentu putanje..
		for (unsigned int j = 0; j < myInfo.grid.xGrid[i].data.size(); j++)
		{
			// pamćenje x i y koordinata trenutne čelije..
			tmpX = myInfo.grid.xGrid[i].data[j];
			tmpY = myInfo.grid.yGrid[i].data[j];

 			// Ako čelija (tmpX,tmpY) još nije posjećena..
			if (li[tmpX][tmpY].size() == 0)
				ti[tmpX][tmpY] = myInfo.grid.timeInGrid[i].data[j] + deltaTime;

			// Ako je vrijeme ulaska u čeliju veće od prošlog maksimalnog izlaznog vremena..
			// .. pamti se novo maksimalno izlazno vrijeme
			if (myInfo.grid.timeInGrid[i].data[j] > max)
				max = myInfo.grid.timeOutGrid[i].data[j];

			// Pamćenje trenutka kada vozilo napušta čeliju
			to[tmpX][tmpY] = myInfo.grid.timeOutGrid[i].data[j] + deltaTime;

			// Pamćenje indeksa segmenta putanje na kojem se vozilo nalazi u ovoj čeliji
			li[tmpX][tmpY].push_back(i);
		}

		deltaTime += max;
	}

	// Petlja po trajektorijama ostalih vozila..
	for (unsigned int k = 0; k < otherVehInfo.size(); k++)
	{
		//Ako je udaljenost od drugog vozila manja od radijusa komunikacije (50 jediničnih duljina)..
		//if(Math.Sqrt(Math.Pow(robots[k].getX() - r.getX(), 2) + Math.Pow(robots[k].getY() - r.getY(), 2)) < 50))
		if(!otherVehInfo[k].inFailure) // ako vozilo nije u kvaru..
		{
			//..i ako se promatrano vozilo kreće i nije u kvaru..
			if (otherVehInfo[k].moving)
			{
				deltaTime = 0; //10 * robots[k].GetTimeToFinish();  ****************************** OVO DORADITI !!!!!!! ***************************

				// Pretraživanje po svim segmentima putanje "k-tog" vozila..
				for (unsigned int i = 0; i < otherVehInfo[k].grid.xGrid.size(); i++)
				{
					max = 0.0;

					// Pretraživanje po svim čelijama kroz koje "k-to" vozilo prolazi na pojedinom segmentu putanje..
					for (unsigned int j = 0; j < otherVehInfo[k].grid.xGrid[i].data.size(); j++)
					{
						// pamćenje x i y koordinata trenutne čelije..
						tmpX = otherVehInfo[k].grid.xGrid[i].data[j];
						tmpY = otherVehInfo[k].grid.yGrid[i].data[j];

						// Ako je vrijeme izlaska "k-tog" vozila iz čelije veće od "max", u "max" se upisuje novo vrijeme
						if (otherVehInfo[k].grid.timeOutGrid[i].data[j] > max)
							max = otherVehInfo[k].grid.timeOutGrid[i].data[j];

						// Ako je sljedeći uvjet zadovoljen, vozilo koje provjerava koliziju..
						// .. i "k-to" vozilo prolaze istom čelijom što može uzrokovati koliziju
						if (li[tmpX][tmpY].size() > 0)
						{
							// Za svaki segment vlastite putanje koji prolazi promatranom čelijom..
							for (unsigned int m = 0; m < li[tmpX][tmpY].size(); m++)
							{
								//fprintf(stderr, "%s detektirao moguću koliziju sa %s, na %d. segmentu!\n", myInfo.vehicleName.c_str(), \
								otherVehInfo[k].vehicleName.c_str(), li[tmpX][tmpY][m]);

								// U listu "ColWithoutTimeMy" upisuju se indeksi vlastitih "kritičnih" segmenata..
								ColWithoutTimeMy[k].push_back(li[tmpX][tmpY][m]);

								// U listu "ColWithoutTimeOther" upisuje se indeks segmenta putanje..
								// .. na kojem "k-to" vozilo prolazi ovom čelijom
								ColWithoutTimeOther[k].push_back(i);

								// Ako postoji vremensko preklapanje zauzetosti promatrane čelije..
								if (overlap(ti[tmpX][tmpY], to[tmpX][tmpY], otherVehInfo[k].grid.timeInGrid[i].data[j] + deltaTime,\
									otherVehInfo[k].grid.timeOutGrid[i].data[j] + deltaTime))
								{
									// U listu "ColWithTimeMy[k]" upisuju se indeks m-tog vlastitog (kritičnog) segmenta..
									ColWithTimeMy[k].push_back(li[tmpX][tmpY][m]);

									// U listu "ColWithTimeOther" upisuje se indeks segmenta putanje..
									// .. na kojem "k-to" vozilo prolazi ovom čelijom
									ColWithTimeOther[k].push_back(i);

									//fprintf(stderr, "%s detektirao vremensku koliziju sa %s, na %d. segmentu!\n", myInfo.vehicleName.c_str(), \
									otherVehInfo[k].vehicleName.c_str(), li[tmpX][tmpY][m]);
								}
							}
						}
					}

				deltaTime += max;

				}
			}

			// Ako "k-to" vozilo miruje..
			else
			{
				// Pretraživanje po svim čelijama koje "k-to" vozilo zauzima u mirovanju..
				for (unsigned int i = 0; i < otherVehInfo[k].xStop.size(); i++)
				{
					// pamćenje x i y koordinata trenutne čelije..
					tmpX = otherVehInfo[k].xStop[i];
					tmpY = otherVehInfo[k].yStop[i];

					// Ako je sljedeći uvjet zadovoljen, to znači da vozilo koje provjerava koliziju..
					// .. i "k-to" vozilo na svom putu prolaze istom čelijom što može uzrokovati koliziju
					if (li[tmpX][tmpY].size() > 0)
					{
						// Za svaki segment vlastite putanje koji prolazi promatranom čelijom..
						for (unsigned int m = 0; m < li[tmpX][tmpY].size(); m++)
						{
							// U liste "ColWithTimeMy" i "ColWithoutTimeMy" upisuje se indeks m-tog (kritičnog) segmenata..
							ColWithTimeMy[k].push_back(li[tmpX][tmpY][m]);
							ColWithoutTimeMy[k].push_back(li[tmpX][tmpY][m]);

							// U liste "ColWithTimeOther" i "ColWithoutTimeOther" za indekse "kritičnih" segmenata "k-tog" vozila..
							// .. upisuje se nula budući da vozilo stoji
							ColWithTimeOther[k].push_back(0);
							ColWithoutTimeOther[k].push_back(0);
						}
					}
				}
			}
		}
	}

	for(int i = 0; i < mapHeight; i++)
	{
		delete [] ti[i];
		delete [] to[i];
	}
}


vector<int> GetColWithTimeMy(int index)
{
	return ColWithTimeMy[index]; ;
}

vector<int> GetColWithoutTimeMy(int index)
{
	return ColWithoutTimeMy[index];
}

vector<int> GetColWithTimeOther(int index)
{
	return ColWithTimeOther[index];
}

vector<int> GetColWithoutTimeOther(int index)
{
	return ColWithoutTimeOther[index];
}

// Provjera kolizije
bool colCheck(const agv_control::vehInfo& myInfo, const vector<agv_control::vehInfo>& otherVehInfo, int mapWidth, int mapHeight, float mapResolution)
{
	vector<int> withTimeMy;
	vector<int> withoutTimeMy;

	vector<int> withTimeOther;
	vector<int> withoutTimeOther;

	int min_wo, min_wok;

	// Detekcija segmenata putanja na kojima dolazi do kolizije s drugim vozilima unutar radiusa komunikacije
	Collision(myInfo, otherVehInfo, mapWidth, mapHeight);

	stop = false;

	// **********************************************************************
	nav_msgs::GridCells gCells;
	gCells.header = myInfo.header;
	gCells.cell_width = mapResolution;
	gCells.cell_height = mapResolution;
	// **********************************************************************

	// Pretraživanje po svim trajektorijama..
	for (unsigned int i = 0; i < otherVehInfo.size(); i++)
	{
		// Učitavanje indeksa segmenata vlastite putanje na kojima može doći do kolozije s "i-tim" vozilom
		withTimeMy = GetColWithTimeMy(i);
		withoutTimeMy = GetColWithoutTimeMy(i);

		// Učitavanje indeksa segmenata putanje "i-tog" vozila na kojima može doći do kolozije
		withTimeOther = GetColWithTimeOther(i);
		withoutTimeOther = GetColWithoutTimeOther(i);

		// Ako je sljedeći uvjet zadovoljen, postoji opasnost od kolizije (jer "withoutTimeMy" varijabla sadrži..
		// ..indekse segmenata vlastite putanje koji prolaze istim čelijama kao i segmenti putanje "i-tog" vozila)
		if (withoutTimeMy.size() > 0)
		{
			agv_control::vehInfo vehInfo_i = otherVehInfo[i];

			// Sortiranje "kritičnih" segmenata oba vozila tako da budu zapisani u rastućem redosljedu..
			sort (withoutTimeMy.begin(), withoutTimeMy.end());
			sort (withoutTimeOther.begin(), withoutTimeOther.end());

			sort (withTimeMy.begin(), withTimeMy.end());

			// indeks prvog sljedećeg segmenta na kojem postoji opasnost od kolizije s "i-tim" vozilom (bez provjere vremenskog preklapanja)
			min_wo = withoutTimeMy[0];
			// indeks prvog sljedećeg segmenta "i-tog" vozila na kojem postoji opasnost od kolizije (bez provjere vremenskog preklapanja)
			min_wok = withoutTimeOther[0];

			// *********** VIZUALIZACIJA ČELIJA S VREMENSKIM PREKLAPANJEM ***********
			for(unsigned int s = 0; s < withTimeMy.size(); s++)
			{
				for(unsigned int j = 0; j < myInfo.grid.xGrid[withTimeMy[s]].data.size(); j++)
				{
					geometry_msgs::Point a;
					a.x = myInfo.grid.xGrid[withTimeMy[s]].data[j] * mapResolution;
					a.y = myInfo.grid.yGrid[withTimeMy[s]].data[j] * mapResolution;
					a.z = 0.0;

					gCells.cells.push_back(a);
				}
			}
			// **********************************************************************

			/*for(int f = 0; f < withoutTimeMy.size(); f++)
			{
				fprintf(stderr, "%s has detected a possible collision with %s on its %d. and his %d. seg!\n", myInfo.vehicleName.c_str(), \
								otherVehInfo[i].vehicleName.c_str(), withoutTimeMy[f], withoutTimeOther[f]);
			}*/

			// Ako je ovaj uvjet zadovoljen znači da postoji vremensko preklapanje zauzetosti čelija kroz koje prolaze oba vozila
			if (withTimeMy.size() > 0)
			{
				// Ako je prioritet "i-tog" vozila manji od prioriteta vozila koje provjerava koliziju (veći broj -> manji prioritet!?)
				if (myInfo.priority < vehInfo_i.priority)
				{
					// Ako "i-to" vozilo stoji ili je sljedeći segment njegove putanje upravo segment na kojem se presjecaju putanje..
					if (min_wok == 0)
					{
						// Zaustavlja se vozilo koje je pokrenulo provjeru kolizije..
						stop = true;

						// LOGIRANJE
						//addToLog(myInfo.vehicleName + " se zaustavio i želi pomaknuti " + vehInfo_i.vehicleName);

						//if (!vehInfo_i.planning && !vehInfo_i.moving && !vehInfo_i.removing && !myInfo.removing)
						//{
							// Ako je broj vozila kojima smeta "i-to" vozilo manji ili jednak broju vozila kojima smeta vozilo kojem smeta "i-to" vozilo..
							if (vehInfo_i.WrongRobots.size() <= myInfo.WrongRobots.size())
							{
								//vehInfo_i.inRemoveMode = true;

								// LOGIRANJE
								//addToLog(myInfo.vehicleName + " izvodi removing nad " + vehInfo_i.vehicleName);

								// Izdavanje zahtjeva za pomicanjem ("removingom") "i-tog" vozila
								sendRemovalRequest(vehInfo_i.vehicleName);
							}
						//}
					}
				}

				// Ako je prioritet "i-tog" vozila veći od prioriteta vozila koje provjerava koliziju
				else
				{
					// Ako do presjecanja putanje dolazi na jednom od sljedeća dva segmenta putanje vozila koje provjerava koliziju..
					if (min_wo <= 1)
					{
						stop = true;

						// LOGIRANJE
						//addToLog(myInfo.vehicleName + " se zaustavio");
					}

					// Ako "i-to" vozilo stoji ili je sljedeći segment njegove putanje upravo segment na kojem se presjecaju putanje..
					if (min_wok == 0) //  && r.xPoints.Count == 0)
					{
						// Zaustavlja se vozilo koje je pokrenulo provjeru kolizije..
						stop = true;

						// LOGIRANJE
						//addToLog(myInfo.vehicleName + " se zaustavio i želi pomaknuti " + vehInfo_i.vehicleName);

						//if (!vehInfo_i.planning && !vehInfo_i.moving && !vehInfo_i.removing && !myInfo.removing)// && r.xPoints.Count == 0)
						//{
							// Ako je broj vozila kojima smeta "i-to" vozilo manji ili jednak broju vozila kojima smeta vozilo kojem smeta "i-to" vozilo..
							if (vehInfo_i.WrongRobots.size() <= myInfo.WrongRobots.size())
							{
								//vehInfo_i.inRemoveMode = true;

								// LOGIRANJE
								//addToLog(myInfo.vehicleName + " izvodi removing nad " + vehInfo_i.vehicleName);

								// Izdavanje zahtjeva za pomicanjem ("removingom") "i-tog" vozila
								sendRemovalRequest(vehInfo_i.vehicleName);
							}
						//}
					}
				}
			}

			// Ako ne postoji vremensko preklapanje zauzetosti čelija kroz koje prolaze oba vozila..
			// .. i ako je indeks prvog sljedećeg segmenta putanje na kojem postoji opasnost od kolizije..
			// ..s "i-tim" vozilom (bez provjere vremenskog preklapanja)..
            // .. manja od 1 za oba vozila i ako se "i-to" vozilo giba..
			else if (min_wo <= 1 && min_wok <= 1 && vehInfo_i.moving)
			{
				// Vozilo se zaustavlja
				//stop = true;
				// LOGIRANJE
				//addToLog(myInfo.vehicleName + " se zaustavio u \"else if\" grani)");
			}
		}
	}

	publishGridCells(gCells);
	return stop;
}

// Određivanje svih zabranjenih čvorova u grafu na koje se ne smije izvršiti pomicanje vozila kod removinga..
// .. jer tuda prolazi (jedno ili više) vozilo koje je zatražilo removing
void GenerateWrongXY(const vector<string>& vehBlockedByMe, const vector<agv_control::vehInfo>& vehInfoList, int mapWidth, int mapHeight, int node_dist, vector<int> &WrongX, vector<int> &WrongY)
{
	bool added[mapWidth][mapHeight];

	for (int i = 0; i < mapWidth; i++)
		for (int j = 0; j < mapHeight; j++)
			added[i][j] = false;

	int tmp;
	vector<int> tmpwx;
	vector<int> tmpwy;

	// Petlja po imenima svih vozila koja su blokirana..
	for (unsigned int k = 0; k < vehBlockedByMe.size(); k++)
	{
		for (unsigned int wri = 0; wri < vehInfoList.size(); wri++)
		{
			if(vehBlockedByMe[k] == vehInfoList[wri].vehicleName)
			{
				agv_control::vehInfo r = vehInfoList[wri];

				// Pretraživanje po svim preostalim segmentima putanje vozila..
				for (unsigned int i = 0; i < r.grid.xGrid.size(); i++)
				{
					// Pretraživanje po svim čelijama kroz koje vozilo prolazi na pojedinom segmentu putanje..
					for (unsigned int j = 0; j < r.grid.xGrid[i].data.size(); j++)
					{
						tmpwx.clear();
						tmpwy.clear();

						// Traže se zabranjeni čvorovi grafa (zato se dijeli s node_dist)
						tmp = r.grid.xGrid[i].data[j] / node_dist;
						tmpwx.push_back(tmp);

						if (node_dist * tmp != r.grid.xGrid[i].data[j])
							tmpwx.push_back((tmp + 1));

						tmp = r.grid.yGrid[i].data[j] / node_dist;
						tmpwy.push_back(tmp);

						if (node_dist * tmp != r.grid.yGrid[i].data[j])
							tmpwy.push_back((tmp + 1));


						for (unsigned int wi = 0; wi < tmpwx.size(); wi++)
						{
							for (unsigned int wj = 0; wj < tmpwy.size(); wj++)
							{
								if (!added[tmpwx[wi]][tmpwy[wj]])
								{
									// Dodavanje x - koordinate zabranjenog čvora u grafu
									WrongX.push_back(tmpwx[wi]);
									// Dodavanje y - koordinate zabranjenog čvora u grafu
									WrongY.push_back(tmpwy[wj]);
									added[tmpwx[wi]][tmpwy[wj]] = true;
								}
							}
						}
					}
				}
				break;
			}
		}
	}
}
