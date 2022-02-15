FROM ros:noetic-robot
RUN apt-get update && apt-get install -y sudo build-essential git pip

# add user with sudo privileges which is not prompted for password
RUN adduser --disabled-password --gecos '' user
RUN adduser user sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER user

RUN pip install casadi-horizon

# WORKDIR /home/daniele
# RUN sudo apt-get update
# RUN sudo apt-get -y upgrade


# # this is required to find conda
# ENV PATH /home/daniele/mambaforge/bin:$PATH
# #RUN conda install mamba -c conda-forge

# RUN mkdir ~/code

# COPY environment.yml /home/daniele/code
# RUN cd ~/code && yes Y | mamba env create -f environment.yml

# # switch to "kindyn" conda environment
# #SHELL ["conda", "run", "-n", "kindyn", "/bin/bash", "-c"]

# #this is not necessary as conda-build will download all the required packages itself
# #RUN cd ~/code && git clone https://github.com/ADVRHumanoids/casadi_kin_dyn.git

# RUN mkdir /home/daniele/code/casadi_kin_dyn
# COPY meta.yaml /home/daniele/code/casadi_kin_dyn
# COPY build.sh /home/daniele/code/casadi_kin_dyn
# COPY initial_commands.sh /home/daniele/

# # add an example for testing
# RUN mkdir /home/daniele/code/examples
# COPY /examples/cart_pole_urdf_reader.py /home/daniele/code/examples
# COPY /examples/cart_pole.urdf /home/daniele/code/examples
