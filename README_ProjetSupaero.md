For the newly uploaded ProjetSupaero, something needs to be specified:

1. For the ProjetSupaero, I have used the input control signal as the only input signal. Of course, when there are the input signal and the optimal control input signal, a trade-off between these two has to be made. This is still to be considered afterwards.

2. I used the RungeKutta to integrate the differential equations that describe the quad-rotor, however, the results seem to be a bit strange. There exits some mistakes in quadModel part or the quadRungeKutta part, I think. This still needs to be checked for the simulation part.

3. For the moment, I used the files in which the model and the RungeKutta are specified, because in the MPCSolver class, these two member functions are marked to be private. Therefore, I can't reach them from outside the class if I have well understood the rules of C++. Besides, I entirely agree to set them to be private, as they should not be visible from outside. Therefore, I choose to use these two functions from outside. This will also be fixed later.

4. For the use of SFML in the context of ProjetSupaero. The only thing two do after installation of SFML is to add the SFML library in the CMakeLists.txt file in the folder src. And before using it in the C++ source file, one should make sure where the SFML library has been installed. That is where to find the corresponding files. Normally, it should be in a folder in /usr, however, it depends.

5. For the model, I recommend to use directly the input signals as the four inputs of the quad-rotor model, to simplify the control. Besides, more simplification need to be made, I think.

6. For the version of SFML, I used the version 2.1 and chef used the version 2.3. For its download, using ubuntu 14.04, I used
  sudo apt-get install libsfml-dev
to get SFML.
