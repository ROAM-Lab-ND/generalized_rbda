using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Pages : MonoBehaviour
{
    private const int numberOfPages = 2;
    private int currentPage = 0;
    private GameObject[] pages = new GameObject[numberOfPages];
    private GameObject pageNumber;
    private Vector3 pageLocation;
    private Vector3 storageLocation;

    void Start() {
      pageNumber = GameObject.Find("Page Number");
      pageNumber.GetComponent<UnityEngine.UI.InputField>().text = (currentPage+1).ToString();
      pageLocation = new Vector3(-43, 246, 0);
      storageLocation = new Vector3(-43, 10000, 0);

      for(int i = 0; i < numberOfPages; i++) {
        pages[i] = GameObject.Find("Page " + (i+1));
        pages[i].transform.localPosition = storageLocation;
      }
      pages[0].transform.localPosition = pageLocation;
    }

    public void pageRight(){
      if(currentPage < numberOfPages-1) {
        pages[currentPage].transform.localPosition = storageLocation;
        currentPage++;
        pages[currentPage].transform.localPosition = pageLocation;
        pageNumber.GetComponent<UnityEngine.UI.InputField>().text = (currentPage+1).ToString();
      }
    }

    public void pageLeft(){
      if(currentPage > 0) {
        pages[currentPage].transform.localPosition = storageLocation;;
        currentPage--;
        pages[currentPage].transform.localPosition = pageLocation;
        pageNumber.GetComponent<UnityEngine.UI.InputField>().text = (currentPage+1).ToString();
      }
    }


    public void setPage(string page) {
      int pageInt = int.Parse(page)-1;
      if(pageInt >= 0 && pageInt < numberOfPages) {
        pages[currentPage].transform.localPosition = storageLocation;
        currentPage = pageInt;
        pages[currentPage].transform.localPosition = pageLocation;
      }
      pageNumber.GetComponent<UnityEngine.UI.InputField>().text = (currentPage+1).ToString();
    }
}
