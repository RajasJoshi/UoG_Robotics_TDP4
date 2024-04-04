import os
import sys
import pandas as pd
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

from Base.SupervisorBase import SupervisorBase
from Utils.Consts import TIME_STEP
from Utils import Functions

class Supervisor(SupervisorBase):
    def __init__(self, file_name: str):
        super().__init__()
        self.file_name = file_name

    def run(self) -> None:
        while super().step(TIME_STEP) != -1:
            super().sendSupervisorData()
            super().isitGoal()
            super().updateScoreboard()
            self.logging()

    def logging(self) -> None:
        data = super().get_data()
        Functions.write_data(self.file_name,data)

    def possession_percentage(self) -> None:
        df = pd.read_csv(self.file_name)
        possession_counts = df.iloc[:, 1].value_counts()

        # Plot the counts in a pie chart
        plt.figure(figsize=(8, 8))
        plt.pie(possession_counts, labels=possession_counts.index, autopct='%1.2f%%', startangle=140)
        plt.title('Distribution of Possession')
        plt.axis('equal')
        plt.savefig(f'{self.file_name[:-4]}_possession_percentage.png')  # Save the plot as an image
    
    def interrupt_passing_percentage(self) -> None:
        df = pd.read_csv(self.file_name)
        df = df.iloc[:,1].dropna()
        bar_width = 0.35
        teams = ['Red Team','Blue Team']
        index = range(len(teams))
        interrupted = {'red': 0, 'blue': 0}
        passed = {'red': 0, 'blue': 0}
        for i in range(1,len(df)):
            if df.iloc[i-1] != df.iloc[i]:
                if df.iloc[i-1][0] == df.iloc[i][0]:
                    if df.iloc[i][0] == 'R':
                        passed['red'] += 1
                    else:
                        passed['blue'] += 1
                else:
                    if df.iloc[i-1][0] == 'R':
                        interrupted['blue'] += 1
                    else:
                        interrupted['red'] +=1
        
        # Plotting
        plt.bar(index, passed.values(), bar_width, label='PASS')
        plt.bar([i + bar_width for i in index], interrupted.values(), bar_width, label='DEFENCE')

        # Customizing the plot
        plt.xlabel('Teams')
        plt.ylabel('Count')
        plt.title('Passing and Defending for Red and Blue Teams')
        plt.xticks([i + bar_width/2 for i in index], teams)
        plt.legend()

        plt.savefig(f'{self.file_name[:-4]}_passing_defence.png')  # Save the plot as an image

if __name__ == '__main__':

    file_name = 'log_game1.csv'
    for i in range(2,10):
        if os.path.exists(file_name):
            file_name = f'log_game{i}.csv'
        else:
            break
    supervisor = Supervisor(file_name)
    supervisor.run()
    supervisor.possession_percentage()
    supervisor.interrupt_passing_percentage()