## CANN basic environment variables and Python environment variable configuration instructions (C++ sample)

### Prerequisites
The Ascend AI software has been deployed on the environment, please refer to the corresponding version of the CANN installation guide in [Link](https://www.hiascend.com/document).


### Installation Precautions
The development environment and running environment are described as follows:
- **Running environment**: The running environment refers to the environment where operators, inference, or training programs run. The running environment must be on the basis of Ascend AI processors.      
- **Development environment**: Used for code development, debugging, and build. The environment can be based on Ascend AI processors or any environment that meets CANN software installation requirements.  

The development environment and the running environment can be co-located on the same server, or they can be set up separately. In separate scenarios, the executable files compiled in the development environment are executed in the running environment, if the operating system architecture of the development environment and the running environment is different, you need to perform cross-compilation in the development environment.     

### Configuration steps

1.  Log in to the environment where CANN software is installed as the running user.

2.  Configure CANN basic environment variables.

    1.  (Optional) Check what environment variables are involved in the set_env.sh script.

         According to the software packages installed in the environment, go to the corresponding installation directory and view the set_env.sh script. Here is an example of the default installation path for common users:

         - If the Ascend-cann-toolkit package is installed on the environment, check the set_env.sh script in the $HOME/Ascend/ascend-toolkit directory;

         - If the Ascend-cann-nnrt package is installed on the environment, check the set_env.sh script in the $HOME/Ascend/nnrt directory.

    2. (Optional) Run the following command to check whether the value of the environment variable is correct.

         For example, if there is an ASCEND_HOME_PATH environment variable in the set_env.sh script, execute the "echo $ASCEND_HOME_PATH" command to check whether the value of the environment variable in the current environment is consistent with the value of the environment variable in the set_env.sh script. If they are consistent, skip to [Configure Python environment variables](#li001); if not, skip to [Set CANN basic environment variables](#li002), and then continue to check Python environment variables.
 
    3.  Set CANN basic environment variables.<a name="li002"></a>

        Software provides scripts for setting process-level environment variables. You can reference the scripts in processes to automatically set environment variables. The environment variables automatically become invalid after the processes end. The default installation path of the common user is used as an example.

        - Configure environment variables when installing the Ascend-cann-toolkit package.
            ```
            . $HOME/Ascend/ascend-toolkit/set_env.sh
            ```

        - Configure environment variables when installing the Ascend-cann-nnrt package.
            ```
            . $HOME/Ascend/nnrt/set_env.sh
            ``` 

        You can also configure permanent environment variables by modifying the ~/.bashrc file. The procedure is as follows:

            (1) Run the vi ~/.bashrc command in any directory as the running user to open the .bashrc file and append the preceding lines to the file.

            (2) Run the :wq! command to save the file and exit.

            (3) Run the source ~/.bashrc command for the modification to take effect immediately.


3.  Configure Python environment variables (take the default installation path of Python 3.7.5 as an example).<a name="li001"></a>

     1. (Optional) Execute the **echo $LD_LIBRARY_PATH** command. If the returned environment variable value contains the lib library configuration of the corresponding Python version (for example, /usr/local/python3.7.5/lib), the The lib library is configured correctly; otherwise, skip to [Set Python environment variables](#li0003).

     2. (Optional) Execute the **echo $PATH** command. If the returned environment variable value contains the bin configuration of the corresponding Python version (for example, /usr/local/python3.7.5/bin), the Python version of the bin The configuration is correct; otherwise, skip to [Set Python environment variables](#li0003).

     3. Set the Python environment variables. <a name="li0003"></a>
         ```
         # Set the Python 3.7.5 library path.
         export LD_LIBRARY_PATH=/usr/local/python3.7.5/lib:$LD_LIBRARY_PATH
         # If multiple Python 3 versions exist in the user environment, use Python 3.7.5.
         export PATH=/usr/local/python3.7.5/bin:$PATH
         ```
