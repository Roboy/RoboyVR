using UnityEngine;
	
	/// <summary>
    /// Describe your class shortly here.
    /// </summary>
	[RequireComponent(typeof(Rigidbody)]
	public class TemplateClass : Monobehaviour
	{
		#region PUBLIC_MEMBER_VARIABLES
		/// <summary>
		/// Describe your property shortly here.
		/// </summary>
		public int SomeProperty 
		{	
			get 
			{
				return m_SomePrivateVariable; 
			}
			set
			{
				if(value > 9000)
				{
					m_SomePrivateVariable = value;
					Debug.Log("This is unbearable");
				}
			}
		}
		
		/// <summary>
		/// Describe your public variable shortly here.
		/// </summary>
		public string SomePublicVariable;
		
		#endregion // PUBLIC_MEMBER_VARIABLES
		
		#region PRIVATE_MEMBER_VARIABLES
		
		/// <summary>
		/// Describe your serialized variable shortly here.
		/// </summary>
		[SerializeField]
		private float m_SomeSerializedVariable;
		
		/// <summary>
		/// Rigidbody component on the object
		/// </summary>
		private Rigidbody m_Rigidbody;
		
		/// <summary>
		/// Describe your private variable shortly here.
		/// </summary>
		private int m_SomePrivateVariable;
		
		#endregion // PRIVATE_MEMBER_VARIABLES
		
		#region UNTIY_MONOBEHAVIOUR_METHODS
		
		/// <summary>
		/// Describe the function shortly here.
		/// </summary>
		void Awake()
		{
			m_Rigidbody = GetComponent<Rigidbody>();
		}
		
		/// <summary>
		/// Describe the function shortly here.
		/// </summary>
		void Start()
		{
			m_SomePrivateVariable = 42;
		}
		
		/// <summary>
		/// Describe the function shortly here.
		/// </summary>
		void OnEnable()
		{
			SomePublicVariable = "Enabled";
		}
		
		/// <summary>
		/// Describe the function shortly here.
		/// </summary>
		void OnDisable()
		{
			SomePublicVariable = "Disabled";
		}
		
		/// <summary>
		/// Describe the function shortly here.
		/// </summary>
		void Update()
		{
			Debug.Log(SomePublicVariable + " : " + m_SomeSerializedVariable);
		}
		
		#endregion // UNTIY_MONOBEHAVIOUR_METHODS
		
		#region PUBLIC_METHODS
		
		/// <summary>
		/// Describe the function shortly here.
		/// </summary>
		public void SomePublicMethod()
		{
			Debug.Log("Do something public!");
		}
		
		/// <summary>
		/// Describe the function shortly here.
		/// </summary>
		public void ActivateBear()
		{
			StartCoroutine(someBearCoroutine());
		}
		
		#endregion // PUBLIC_METHODS
		
		#region PRIVATE_METHODS
		
		/// <summary>
		/// Describe the function shortly here.
		/// </summary>
		private void somePrivateMethod()
		{
			Debug.Log("Do something private!");
		}
		
		/// <summary>
		/// Describe the coroutine shortly here.
		/// </summary>
		private IEnumerator someBearCoroutine()
		{
			while(true)
			{
				Debug.Log("Bear is running!");
				yield return null;
			}
		}
		
		#endregion // PRIVATE_METHODS
	}